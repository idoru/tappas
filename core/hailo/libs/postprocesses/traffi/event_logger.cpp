#include "event_logger.hpp"
#include <sstream>
#include <fstream>
#include <iostream>

std::unique_ptr<EventLogger> EventLogger::instance_ = nullptr;
std::mutex EventLogger::mutex_;

static const std::string histoQuery = "from(bucket: \"traffi\")"
  "|> range(start: -24h)"
  "|> filter(fn: (r) => r._measurement == \"crossing\")"
  "|> group(columns: [\"origin\", \"side\", \"legal\"])"
  "|> aggregateWindow(every: 1h, fn: count, createEmpty: true)"
  "|> drop(columns: [\"_start\", \"_stop\"])";

size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    return size * nmemb;
}

size_t WriteDataCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    size_t totalSize = size * nmemb;
    std::string* response = static_cast<std::string*>(userp);
    response->append(static_cast<char*>(contents), totalSize);
    return totalSize;
}

EventLogger& EventLogger::getInstance() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (instance_ == nullptr) {
        instance_.reset(new EventLogger());
    }
    return *instance_;
}

EventLogger::EventLogger() : running_(true) {
    host_ = getEnvOrDefault("EVENT_DB_HOST", "http://localhost:8086");
    org_ = getEnvOrDefault("EVENT_DB_ORG", "traffi");
    bucket_ = getEnvOrDefault("EVENT_DB_BUCKET", "traffi");
    token_ = getEnvOrDefault("EVENT_DB_TOKEN", "");

    if (token_.empty()) {
        throw std::runtime_error("EVENT_DB_TOKEN environment variable must be set");
    }

    curl_ = curl_easy_init();
    if (!curl_) {
        throw std::runtime_error("Failed to initialize CURL");
    }
    worker_thread_ = std::thread(&EventLogger::processEvents, this);
}

std::string EventLogger::getEnvOrDefault(const char* env_var, const std::string& default_val) {
    const char* val = std::getenv(env_var);
    return val ? std::string(val) : default_val;
}

EventLogger::~EventLogger() {
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        running_ = false;
    }
    queue_cv_.notify_one();

    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }

    if (curl_) {
        curl_easy_cleanup(curl_);
    }
}

bool EventLogger::logDetection(int vehicle_id, const std::string& side) {
    std::stringstream ss;
    ss << "detection,side=" << side
       << " vehicle_id=" << vehicle_id;

    enqueueEvent(ss.str());
    return true;
}

bool EventLogger::logCrossing(int vehicle_id, const std::string& side,
                            const std::string& origin, bool legal) {
    std::stringstream ss;
    ss << "crossing,side=" << side
       << ",origin=" << origin
       << ",legal=" << (legal ? "true" : "false")
       << " vehicle_id=" << vehicle_id;

    enqueueEvent(ss.str());
    return true;
}

void EventLogger::enqueueEvent(const std::string& data) {
    LogEvent event{data, std::chrono::system_clock::now()};
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        event_queue_.push(std::move(event));
    }
    queue_cv_.notify_one();
}

void EventLogger::processEvents() {
    while (true) {
        LogEvent event;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this] {
                return !running_ || !event_queue_.empty();
            });

            if (!running_ && event_queue_.empty()) {
                break;
            }

            event = std::move(event_queue_.front());
            event_queue_.pop();
        }

        sendEvent(event);
    }
}

bool EventLogger::sendEvent(const LogEvent& event) {
    if (!curl_) {
        return false;
    }

    curl_easy_reset(curl_);
    auto nanos = std::chrono::duration_cast<std::chrono::seconds>(
        event.timestamp.time_since_epoch()).count();

    std::string data = event.data + " " + std::to_string(nanos);
    std::string url = host_ + "/api/v2/write?org=traffi&bucket=traffi&precision=s";

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, ("Authorization: Token " + token_).c_str());
    headers = curl_slist_append(headers, "Content-Type: text/plain; charset=utf-8");

    curl_easy_setopt(curl_, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl_, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl_, CURLOPT_POSTFIELDS, data.c_str());
    curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl_, CURLOPT_CUSTOMREQUEST, "POST");

    CURLcode res = curl_easy_perform(curl_);
    curl_slist_free_all(headers);
    if (res == CURLE_OK) {
      long http_code;
      res = curl_easy_getinfo(curl_, CURLINFO_RESPONSE_CODE, &http_code);
      if (res == CURLE_OK && http_code / 100 != 2) {
          std::cout << "POST event FAILED HTTP Status: " << http_code
                    << " DATA: " << data << std::endl;
      }
      if (res == CURLE_OK && http_code / 100 == 2) {
        if (event.data.length() >= 9 && event.data[0] == 'c') {//something was going weird with string checking, but this is enough to distinguish between detections and crossings
          if (!updateReportData(histoQuery, "/var/local/traffi/www/stats.csv")) {
            std::cout << "ERROR updating crossing report" << std::endl;
          }
        }
        return true;
      }
      return false;
    }
    return false;
}

bool EventLogger::updateReportData(const std::string query, const std::string reportPath) {
    if (!curl_) {
        std::cout << "Failed to initialize CURL for updateReportData" << std::endl;
        return false;
    }

    curl_easy_reset(curl_);
    std::string url = host_ + "/api/v2/query?org=traffi";
    curl_easy_setopt(curl_, CURLOPT_URL, url.c_str());


    struct curl_slist* headers = nullptr;
    std::string authHeader = "Authorization: Token " + token_;
    headers = curl_slist_append(headers, authHeader.c_str());
    headers = curl_slist_append(headers, "Accept: application/csv");
    headers = curl_slist_append(headers, "Content-Type: application/vnd.flux");
    curl_easy_setopt(curl_, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl_, CURLOPT_POSTFIELDS, query.c_str());
    curl_easy_setopt(curl_, CURLOPT_CUSTOMREQUEST, "POST");

    std::string response_data;
    curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, WriteDataCallback);
    curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &response_data);

    CURLcode res = curl_easy_perform(curl_);
    if (res != CURLE_OK) {
        std::cout << "POST query failed: " << curl_easy_strerror(res) << std::endl;
        curl_slist_free_all(headers);
        return false;
    }

    long http_code = 0;
    res = curl_easy_getinfo(curl_, CURLINFO_RESPONSE_CODE, &http_code);
    if (res != CURLE_OK) {
        std::cout << "Failed to get HTTP response code: " << curl_easy_strerror(res) << std::endl;
        curl_slist_free_all(headers);
        return false;
    }
    if (http_code / 100 != 2) {
        std::cout << "POST query FAILED HTTP Status: " << http_code
                  << " Query: " << query << std::endl;
        curl_slist_free_all(headers);
        return false;
    }

    curl_slist_free_all(headers);

    {
      static std::mutex file_write_mutex;
      std::lock_guard<std::mutex> lock(file_write_mutex);

      std::ofstream outfile(reportPath, std::ios::out | std::ios::trunc);
      if (!outfile.is_open()) {
          std::cout << "Failed to open file for writing: " << reportPath << std::endl;
          return false;
      }
      outfile << response_data;
      if (!outfile.good()) {
          std::cout << "Failed to write response to file: " << reportPath << std::endl;
          outfile.close();
          return false;
      }
      outfile.close();
    }

    curl_easy_reset(curl_);
    std::string publish_url = "http://localhost/publish";
    curl_easy_setopt(curl_, CURLOPT_URL, publish_url.c_str());
    curl_easy_setopt(curl_, CURLOPT_POSTFIELDS, response_data.c_str());
    curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl_, CURLOPT_CUSTOMREQUEST, "POST");

    res = curl_easy_perform(curl_);
    if (res != CURLE_OK) {
        std::cout << "POST to /publish failed: " << curl_easy_strerror(res) << std::endl;
        return false;
    }

    http_code = 0;
    res = curl_easy_getinfo(curl_, CURLINFO_RESPONSE_CODE, &http_code);
    if (res != CURLE_OK || http_code / 100 != 2) {
        std::cout << "POST to /publish FAILED HTTP Status: " << http_code << std::endl;
        return false;
    }

    return true;
}

#include "event_logger.hpp"
#include <sstream>
#include <iostream>

std::unique_ptr<EventLogger> EventLogger::instance_ = nullptr;
std::mutex EventLogger::mutex_;

size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    return size * nmemb;
}

EventLogger& EventLogger::getInstance() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (instance_ == nullptr) {
        instance_.reset(new EventLogger());
    }
    return *instance_;
}

EventLogger::EventLogger() : running_(true) {
    curl_ = curl_easy_init();
    if (!curl_) {
        throw std::runtime_error("Failed to initialize CURL");
    }
    worker_thread_ = std::thread(&EventLogger::processEvents, this);
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
    ss << "detection,side=\"" << side
       << "\" vehicle_id=" << vehicle_id;

    enqueueEvent(ss.str());
    return true;
}

bool EventLogger::logCrossing(int vehicle_id, const std::string& side,
                            const std::string& origin, bool legal) {
    std::stringstream ss;
    ss << "crossing,side=\"" << side
       << "\",origin=\"" << origin
       << "\",legal=" << (legal ? "true" : "false")
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
        return (res == CURLE_OK && http_code / 100 == 2);
    }
    return false;
}

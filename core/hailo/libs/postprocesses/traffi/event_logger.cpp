#include "event_logger.hpp"
#include <sstream>
#include <iostream>
#include <chrono>
#include <ctime>
std::unique_ptr<EventLogger> EventLogger::instance_ = nullptr;
std::mutex EventLogger::mutex_;

// Static callback for CURL
size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    return size * nmemb;  // Just discard the response
}

EventLogger& EventLogger::getInstance() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (instance_ == nullptr) {
        instance_.reset(new EventLogger());
    }
    return *instance_;
}

EventLogger::EventLogger() {
    curl_ = curl_easy_init();
    if (!curl_) {
        throw std::runtime_error("Failed to initialize CURL");
    }
}

EventLogger::~EventLogger() {
    if (curl_) {
        curl_easy_cleanup(curl_);
    }
}

bool EventLogger::logDetection(int vehicle_id,
                         const std::string& side) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto now = std::chrono::system_clock::now();
    auto nanos = std::chrono::duration_cast<std::chrono::seconds>(
        now.time_since_epoch()).count();

    std::stringstream ss;
    ss << "detection,side=\"" << side
      << "\" vehicle_id=" << vehicle_id << " " << nanos;

    return curl(ss.str());
}

bool EventLogger::logCrossing(int vehicle_id,
                         const std::string& side,
                         const std::string& origin,
                         bool legal) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto now = std::chrono::system_clock::now();
    auto nanos = std::chrono::duration_cast<std::chrono::seconds>(
        now.time_since_epoch()).count();

    std::stringstream ss;
    ss << "crossing,side=\"" << side
       << "\",origin=\"" << origin
       << "\",legal=" << (legal ? "true" : "false")
       << " vehicle_id=" << vehicle_id
       << " " << nanos;

    return curl(ss.str());
}

bool EventLogger::curl(const std::string& data) {
    if (!curl_) {
        return false;
    }

    // Construct the URL
    std::string url = host_ + "/api/v2/write?org=traffi&bucket=traffi&precision=s";

    // Set up the request
    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, ("Authorization: Token " + token_).c_str());
    headers = curl_slist_append(headers, "Content-Type: text/plain; charset=utf-8");

    curl_easy_setopt(curl_, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl_, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl_, CURLOPT_POSTFIELDS, data.c_str());
    curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl_, CURLOPT_CUSTOMREQUEST, "POST");

    // Perform the request
    CURLcode res = curl_easy_perform(curl_);

    // Clean up
    curl_slist_free_all(headers);

    if (res == CURLE_OK) {
        long http_code;
        res = curl_easy_getinfo(curl_, CURLINFO_RESPONSE_CODE, &http_code);
        std::cout << "HTTP POST result " << http_code << std::endl;
        return (res == CURLE_OK && http_code / 100 == 2);
    };
    return false;
}

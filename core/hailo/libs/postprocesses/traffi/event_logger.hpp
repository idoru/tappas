#pragma once

#include <string>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <condition_variable>
#include <curl/curl.h>

struct LogEvent {
    std::string data;
    std::chrono::system_clock::time_point timestamp;
};

class EventLogger {
public:
    static EventLogger& getInstance();
    ~EventLogger();

    EventLogger(const EventLogger&) = delete;
    EventLogger& operator=(const EventLogger&) = delete;

    bool logDetection(int vehicle_id, const std::string& side);
    bool logCrossing(int vehicle_id, const std::string& side,
                    const std::string& origin, bool legal);
    bool updateReportData(const std::string query, const std::string reportPath);

private:
    EventLogger();
    void processEvents();
    bool sendEvent(const LogEvent& event);
    void enqueueEvent(const std::string& data);
    std::string getEnvOrDefault(const char* env_var, const std::string& default_val);

    static std::unique_ptr<EventLogger> instance_;
    static std::mutex mutex_;

    std::string host_;
    std::string org_;
    std::string bucket_;
    std::string token_;

    std::queue<LogEvent> event_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread worker_thread_;
    bool running_;
    CURL* curl_;
};

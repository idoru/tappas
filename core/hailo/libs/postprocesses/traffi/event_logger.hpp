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

private:
    EventLogger();
    void processEvents();
    bool sendEvent(const LogEvent& event);
    void enqueueEvent(const std::string& data);

    static std::unique_ptr<EventLogger> instance_;
    static std::mutex mutex_;

    std::string host_{"http://localhost:8086"};
    std::string org_{"traffi"};
    std::string bucket_{"traffi"};
    std::string token_{"GrdnCU9nnJ_WLXk6yPtRup18-qYUDLL923tzGQz4zjvNLNoM1w1UbKtZZL9-hMOOz3YoC1rILbi2nufy4UQ61g=="};

    std::queue<LogEvent> event_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread worker_thread_;
    bool running_;
    CURL* curl_;
};

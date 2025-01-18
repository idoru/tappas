// event_logger.hpp
#pragma once

#include <string>
#include <memory>
#include <mutex>
#include <curl/curl.h>

class EventLogger {
public:
    static EventLogger& getInstance();
    ~EventLogger();

    // Delete copy constructor and assignment operator
    EventLogger(const EventLogger&) = delete;
    EventLogger& operator=(const EventLogger&) = delete;

    // logging methods
    bool logDetection(int vehicle_id,
                const std::string& side);
    bool logCrossing(int vehicle_id,
                const std::string& side,
                const std::string& origin,
                bool legal);

    // Configure the logger
    // void configure(const std::string& host,
    //               const std::string& org,
    //               const std::string& bucket,
    //               const std::string& token);

private:
    EventLogger();

    bool curl(const std::string& body);
    static std::unique_ptr<EventLogger> instance_;
    static std::mutex mutex_;

    // InfluxDB connection details
    std::string host_{"http://localhost:8086"};
    std::string org_{"traffi"};
    std::string bucket_{"traffi"};
    std::string token_{"GrdnCU9nnJ_WLXk6yPtRup18-qYUDLL923tzGQz4zjvNLNoM1w1UbKtZZL9-hMOOz3YoC1rILbi2nufy4UQ61g=="};

    // CURL handle
    CURL* curl_;
};

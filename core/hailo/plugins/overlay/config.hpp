#pragma once

#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <atomic>
#include <optional>
#include <opencv2/opencv.hpp>

class Config {
public:
    struct ConfigEntry {
        float p0x, p0y, p1x, p1y;
        cv::Scalar color;
        bool testsbelow;
        bool show, showcar;
        std::string label;
        std::vector<std::string> prohibited;
    };

    // Delete copy and move operations
    Config(const Config&) = delete;
    Config& operator=(const Config&) = delete;
    Config(Config&&) = delete;
    Config& operator=(Config&&) = delete;

    // Singleton access
    static Config& Get() {
        static Config instance;
        return instance;
    }

    // Thread-safe access to config entries
    std::vector<ConfigEntry> GetEntries() const;

private:
    static constexpr const char* DEFAULT_CONFIG_PATH = "/etc/traffi/boundaries.csv";

    // Private constructor for singleton
    Config() {
        loadConfig();
        startFileWatcher();
    }
    ~Config();

    void startFileWatcher();
    void cleanupWatcher();
    std::optional<ConfigEntry> parseLine(const std::string& line);
    bool loadConfig();

    std::string filepath_{DEFAULT_CONFIG_PATH};
    std::vector<ConfigEntry> entries_;
    mutable std::shared_mutex mutex_;
    std::optional<std::thread> watcher_thread_;
    std::atomic<bool> should_run_{false};
};

#include "config.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>
#include <unistd.h>
#include <sys/inotify.h>

Config::~Config() {
    cleanupWatcher();
}

std::vector<Config::ConfigEntry> Config::GetEntries() const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return entries_;
}

void Config::cleanupWatcher() {
    should_run_ = false;
    if (watcher_thread_ && watcher_thread_->joinable()) {
        watcher_thread_->join();
        watcher_thread_.reset();
    }
}

void Config::startFileWatcher() {
    should_run_ = true;
    watcher_thread_.emplace([this]() {
        const int inotify_fd = inotify_init1(IN_NONBLOCK);
        if (inotify_fd < 0) {
            std::cerr << "Failed to initialize inotify" << std::endl;
            return;
        }

        const int watch_fd = inotify_add_watch(
            inotify_fd,
            std::filesystem::path(filepath_).parent_path().c_str(),
            IN_CLOSE_WRITE
        );

        if (watch_fd < 0) {
            close(inotify_fd);
            std::cerr << "Failed to add watch" << std::endl;
            return;
        }

        constexpr size_t event_size = sizeof(struct inotify_event);
        constexpr size_t buf_len = (event_size + 256) * 16;
        char buffer[buf_len];

        while (should_run_) {
            const ssize_t len = read(inotify_fd, buffer, buf_len);
            if (len < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }
                if (errno == EINTR) continue;

                std::cerr << "Error reading inotify events" << std::endl;
                break;
            }

            for (char* ptr = buffer; ptr < buffer + len;) {
                auto* event = reinterpret_cast<struct inotify_event*>(ptr);
                if (event->mask & IN_CLOSE_WRITE) {
                    if (std::filesystem::path(filepath_).filename() == event->name) {
                        std::cout << "Config file changed, reloading..." << std::endl;
                        loadConfig();
                    }
                }
                ptr += event_size + event->len;
            }
        }

        inotify_rm_watch(inotify_fd, watch_fd);
        close(inotify_fd);
    });
}

std::optional<Config::ConfigEntry> Config::parseLine(const std::string& line) {
    std::istringstream ss(line);
    std::string token;
    ConfigEntry entry;

    try {
        // Parse floating point values
        if (!(ss >> entry.p0x)) return std::nullopt;
        if (ss.get() != ',') return std::nullopt;
        if (!(ss >> entry.p0y)) return std::nullopt;
        if (ss.get() != ',') return std::nullopt;
        if (!(ss >> entry.p1x)) return std::nullopt;
        if (ss.get() != ',') return std::nullopt;
        if (!(ss >> entry.p1y)) return std::nullopt;
        if (ss.get() != ',') return std::nullopt;

        // Parse RGB values
        int r, g, b;
        if (!(ss >> r)) return std::nullopt;
        if (ss.get() != ',') return std::nullopt;
        if (!(ss >> g)) return std::nullopt;
        if (ss.get() != ',') return std::nullopt;
        if (!(ss >> b)) return std::nullopt;
        if (ss.get() != ',') return std::nullopt;

        // Validate RGB ranges
        if (r < 0 || r > 255 || g < 0 || g > 255 || b < 0 || b > 255) {
            return std::nullopt;
        }
        entry.color = cv::Scalar(r, g, b);  // OpenCV uses BGR order but weirdly HAILO uses RGB

        // Parse boolean
        int test_below;
        if (!(ss >> test_below)) return std::nullopt;
        if (ss.get() != ',') return std::nullopt;
        entry.testsbelow = (test_below != 0);

        int show;
        if (!(ss >> show)) return std::nullopt;
        if (ss.get() != ',') return std::nullopt;
        entry.show = (show != 0);

        int showcar;
        if (!(ss >> showcar)) return std::nullopt;
        if (ss.get() != ',') return std::nullopt;
        entry.showcar = (showcar != 0);

        // Parse label
        std::getline(ss, token, ',');
        if (token.empty()) return std::nullopt;
        entry.label = token;

        // Parse prohibited directions
        std::string prohibited_str;
        std::getline(ss, prohibited_str);
        std::istringstream prohibited_ss(prohibited_str);
        std::string direction;
        while (std::getline(prohibited_ss, direction, '|')) {
            if (!direction.empty()) {
                entry.prohibited.push_back(direction);
            }
        }

        return entry;
    } catch (const std::exception& e) {
        std::cerr << "Error parsing line: " << e.what() << std::endl;
        return std::nullopt;
    }
}

bool Config::loadConfig() {
    std::vector<ConfigEntry> new_entries;
    std::ifstream file(filepath_);

    if (!file.is_open()) {
        std::cerr << "Failed to open config file: " << filepath_ << std::endl;
        return false;
    }

    std::string line;
    size_t line_number = 0;
    bool has_error = false;

    while (std::getline(file, line)) {
        line_number++;
        if (line.empty() || line[0] == '#') {
            continue;
        }

        auto entry = parseLine(line);
        if (!entry) {
            std::cerr << "Error parsing line " << line_number << ": " << line << std::endl;
            has_error = true;
            continue;
        }
        new_entries.push_back(*entry);
    }

    if (has_error) {
        return false;
    }

    // Update the entries atomically
    std::unique_lock<std::shared_mutex> lock(mutex_);
    entries_ = std::move(new_entries);
    return true;
}

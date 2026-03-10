#include "runtime_log.h"

#include <fstream>
#include <iostream>
#include <mutex>

namespace runtime_log {
namespace {
std::mutex g_log_mutex;
std::string g_log_path;
}

void setLogFile(const std::string& path) {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    g_log_path = path;
}

void logLine(const std::string& message) {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    std::cout << message << "\n";
    if (!g_log_path.empty()) {
        std::ofstream ofs(g_log_path, std::ios::app);
        if (ofs.is_open()) {
            ofs << message << "\n";
        }
    }
}

void logLineFileOnly(const std::string& message) {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    if (!g_log_path.empty()) {
        std::ofstream ofs(g_log_path, std::ios::app);
        if (ofs.is_open()) {
            ofs << message << "\n";
        }
    }
}

} // namespace runtime_log

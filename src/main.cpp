#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <memory>
#include <atomic>
#include <chrono>
#include <cstring>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <limits>
#include <cstdint>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <fcntl.h>
#include <cerrno>
#include <opencv2/opencv.hpp>
#include "data_recv.h"
#include "frame_processor.h"
#include "thread_affinity.h"
#include "runtime_log.h"

#include "config.h"

namespace {
constexpr std::size_t QUEUE_CAP_B_INPUT = 16;
constexpr std::size_t QUEUE_CAP_C_INPUT = 16;
constexpr std::size_t QUEUE_CAP_B_OUTPUT = 32;
constexpr std::size_t QUEUE_CAP_C_OUTPUT = 32;
constexpr std::size_t QUEUE_CAP_CHAPARIS = 32;
constexpr std::size_t QUEUE_CAP_DS = 32;
constexpr std::size_t QUEUE_CAP_VISUAL = 32;
constexpr std::size_t FRAME_MAP_CAP = 64;

pid_t currentThreadTid() {
    return static_cast<pid_t>(::syscall(SYS_gettid));
}

template <typename T>
void trimFrameMapByOldestKey(std::unordered_map<uint16_t, T>& map, std::size_t max_size) {
    while (map.size() > max_size) {
        auto oldest_it = std::min_element(
            map.begin(),
            map.end(),
            [](const auto& a, const auto& b) {
                return a.first < b.first;
            });
        if (oldest_it == map.end()) {
            break;
        }
        map.erase(oldest_it);
    }
}

bool popOldestFrameId(std::unordered_map<uint16_t, std::shared_ptr<CompleteFrame>>& map, uint16_t& frame_id) {
    if (map.empty()) {
        return false;
    }
    auto oldest_it = std::min_element(
        map.begin(),
        map.end(),
        [](const auto& a, const auto& b) {
            return a.first < b.first;
        });
    if (oldest_it == map.end()) {
        return false;
    }
    frame_id = oldest_it->first;
    map.erase(oldest_it);
    return true;
}

class RuntimeResourceMonitor {
public:
    explicit RuntimeResourceMonitor(std::string log_path,
        std::chrono::milliseconds sample_interval = std::chrono::milliseconds(100))
        : log_path_(std::move(log_path)),
          sample_interval_(sample_interval),
          clock_ticks_per_second_(::sysconf(_SC_CLK_TCK)),
          monitor_start_time_(std::chrono::steady_clock::now()) {
        std::ofstream clear_ofs(log_path_, std::ios::trunc);
        if (clear_ofs.is_open()) {
            clear_ofs << "========== Runtime Resource Usage ==========" << "\n";
            clear_ofs << "sample_interval_ms=" << sample_interval_.count() << "\n";
            clear_ofs << "metric_note: sample/average over frame window [full frame received -> visualization completed]" << "\n";
        }
    }

    void registerThread(const std::string& thread_name, pid_t tid, int bind_core_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto& record = threads_[tid];
        record.name = thread_name;
        record.bind_core_id = bind_core_id;
    }

    void start(const std::atomic<bool>& stop_flag) {
        monitor_thread_ = std::thread([this, &stop_flag]() {
            auto last_time = std::chrono::steady_clock::now();
            while (!stop_flag.load()) {
                std::this_thread::sleep_for(sample_interval_);
                auto now = std::chrono::steady_clock::now();
                const double elapsed_seconds = std::chrono::duration<double>(now - last_time).count();
                if (elapsed_seconds > 0.0) {
                    sampleOnce(elapsed_seconds, now);
                }
                last_time = now;
            }
        });
    }

    void onFrameWindowStart(uint16_t frame_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        active_frame_ids_.insert(frame_id);
    }

    void onFrameWindowEnd(uint16_t frame_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        active_frame_ids_.erase(frame_id);
    }

    void stopAndDump() {
        if (monitor_thread_.joinable()) {
            monitor_thread_.join();
        }
        dumpToLog();
    }

private:
    struct ThreadRecord {
        std::string name;
        int bind_core_id = -1;
        uint64_t last_total_ticks = 0;
        bool has_last_ticks = false;
        double cpu_util_sum_in_window = 0.0;
        uint64_t cpu_sample_count_in_window = 0;
        double vmrss_kb_sum_in_window = 0.0;
        uint64_t mem_sample_count_in_window = 0;
    };

    struct CoreRecord {
        double cpu_util_sum_in_window = 0.0;
        uint64_t sample_count_in_window = 0;
    };

    struct ThreadSample {
        uint64_t sample_index = 0;
        double elapsed_ms = 0.0;
        std::string thread_name;
        int core_id = -1;
        double cpu_util = 0.0;
        double vmrss_kb = 0.0;
    };

    void appendSampleLocked(const ThreadSample& sample) {
        std::ofstream ofs(log_path_, std::ios::app);
        if (!ofs.is_open()) {
            return;
        }
        if (!sample_section_written_) {
            ofs << "\n[Per-Sample In Frame Window]\n";
            sample_section_written_ = true;
        }
        ofs << std::fixed << std::setprecision(2)
            << "sample=" << sample.sample_index
            << " t_ms=" << sample.elapsed_ms
            << " thread=" << sample.thread_name
            << " core=" << sample.core_id
            << " cpu=" << sample.cpu_util << "%"
            << " vmrss=" << sample.vmrss_kb << "KB"
            << "\n";
    }

    bool readThreadStat(pid_t tid, uint64_t& total_ticks, int& processor_id) const {
        std::ifstream stat_file("/proc/self/task/" + std::to_string(tid) + "/stat");
        if (!stat_file.is_open()) {
            return false;
        }

        std::string line;
        std::getline(stat_file, line);
        if (line.empty()) {
            return false;
        }

        const std::size_t comm_end = line.rfind(")");
        if (comm_end == std::string::npos || comm_end + 2 >= line.size()) {
            return false;
        }

        std::istringstream iss(line.substr(comm_end + 2));
        std::vector<std::string> fields;
        std::string token;
        while (iss >> token) {
            fields.push_back(token);
        }

        if (fields.size() <= 36) {
            return false;
        }

        try {
            const uint64_t utime_ticks = static_cast<uint64_t>(std::stoull(fields[11]));
            const uint64_t stime_ticks = static_cast<uint64_t>(std::stoull(fields[12]));
            total_ticks = utime_ticks + stime_ticks;
            processor_id = std::stoi(fields[36]);
        } catch (...) {
            return false;
        }

        return true;
    }

    bool readThreadVmRssKb(pid_t tid, double& vmrss_kb) const {
        std::ifstream status_file("/proc/self/task/" + std::to_string(tid) + "/status");
        if (!status_file.is_open()) {
            return false;
        }

        std::string line;
        while (std::getline(status_file, line)) {
            if (line.rfind("VmRSS:", 0) == 0) {
                std::istringstream iss(line);
                std::string key;
                double value = 0.0;
                std::string unit;
                iss >> key >> value >> unit;
                vmrss_kb = value;
                return true;
            }
        }
        return false;
    }

    void sampleOnce(double elapsed_seconds, const std::chrono::steady_clock::time_point& now) {
        bool in_frame_window = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            in_frame_window = !active_frame_ids_.empty();
        }

        std::vector<pid_t> tids;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            tids.reserve(threads_.size());
            for (const auto& kv : threads_) {
                tids.push_back(kv.first);
            }
        }

        for (pid_t tid : tids) {
            uint64_t total_ticks = 0;
            int processor_id = -1;
            const bool stat_ok = readThreadStat(tid, total_ticks, processor_id);

            double vmrss_kb = 0.0;
            const bool mem_ok = readThreadVmRssKb(tid, vmrss_kb);

            std::lock_guard<std::mutex> lock(mutex_);
            auto it = threads_.find(tid);
            if (it == threads_.end()) {
                continue;
            }
            auto& record = it->second;

            if (stat_ok) {
                if (record.has_last_ticks && clock_ticks_per_second_ > 0) {
                    const uint64_t delta_ticks = total_ticks - record.last_total_ticks;
                    const double cpu_util =
                        (static_cast<double>(delta_ticks) * 100.0) /
                        (static_cast<double>(clock_ticks_per_second_) * elapsed_seconds);
                    if (in_frame_window) {
                        record.cpu_util_sum_in_window += cpu_util;
                        record.cpu_sample_count_in_window++;

                        ThreadSample sample;
                        sample.sample_index = ++sample_seq_;
                        sample.elapsed_ms = std::chrono::duration<double, std::milli>(now - monitor_start_time_).count();
                        sample.thread_name = record.name;
                        sample.core_id = record.bind_core_id;
                        sample.cpu_util = cpu_util;
                        sample.vmrss_kb = mem_ok ? vmrss_kb : 0.0;
                        appendSampleLocked(sample);
                    }

                    if (record.bind_core_id >= 0) {
                        auto& core_record = core_stats_[record.bind_core_id];
                        if (in_frame_window) {
                            core_record.cpu_util_sum_in_window += cpu_util;
                            core_record.sample_count_in_window++;
                        }
                    }
                }
                record.last_total_ticks = total_ticks;
                record.has_last_ticks = true;
                (void)processor_id;
            }

            if (mem_ok) {
                if (in_frame_window) {
                    record.vmrss_kb_sum_in_window += vmrss_kb;
                    record.mem_sample_count_in_window++;
                }
            }
        }
    }

    void dumpToLog() {
        std::vector<ThreadRecord> thread_records;
        std::vector<std::pair<int, CoreRecord>> core_records;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            thread_records.reserve(threads_.size());
            for (const auto& kv : threads_) {
                thread_records.push_back(kv.second);
            }
            core_records.reserve(core_stats_.size());
            for (const auto& kv : core_stats_) {
                core_records.push_back(kv);
            }
        }

        std::sort(thread_records.begin(), thread_records.end(), [](const ThreadRecord& a, const ThreadRecord& b) {
            return a.name < b.name;
        });
        std::sort(core_records.begin(), core_records.end(),
                  [](const std::pair<int, CoreRecord>& a, const std::pair<int, CoreRecord>& b) {
                      return a.first < b.first;
                  });

        std::ofstream ofs(log_path_, std::ios::app);
        if (!ofs.is_open()) {
            std::cerr << "[WARN] failed to open resource log: " << log_path_ << "\n";
            return;
        }

        ofs << "[Per-Thread Average In Frame Window]\n";
        for (const auto& record : thread_records) {
            const double avg_cpu = (record.cpu_sample_count_in_window > 0)
                                       ? (record.cpu_util_sum_in_window / static_cast<double>(record.cpu_sample_count_in_window))
                                       : 0.0;
            const double avg_mem = (record.mem_sample_count_in_window > 0)
                                       ? (record.vmrss_kb_sum_in_window / static_cast<double>(record.mem_sample_count_in_window))
                                       : 0.0;
            ofs << "thread=" << record.name
                << " core=" << record.bind_core_id
                << " avg_cpu=" << avg_cpu << "%"
                << " avg_vmrss=" << avg_mem << "KB"
                << " cpu_samples=" << record.cpu_sample_count_in_window
                << " mem_samples=" << record.mem_sample_count_in_window
                << "\n";
        }

        ofs << "[Per-Core Average In Frame Window]\n";
        for (const auto& kv : core_records) {
            const int core_id = kv.first;
            const auto& core = kv.second;
            const double avg_core_util = (core.sample_count_in_window > 0)
                                             ? (core.cpu_util_sum_in_window / static_cast<double>(core.sample_count_in_window))
                                             : 0.0;
            ofs << "core=" << core_id
                << " avg_cpu=" << avg_core_util << "%"
                << " samples=" << core.sample_count_in_window
                << "\n";
        }
        ofs << "===========================================\n";

        std::cout << "[INFO] resource usage log saved: " << log_path_ << "\n";
    }

    std::string log_path_;
    std::chrono::milliseconds sample_interval_;
    long clock_ticks_per_second_ = 100;
    std::thread monitor_thread_;
    mutable std::mutex mutex_;
    std::unordered_map<pid_t, ThreadRecord> threads_;
    std::unordered_map<int, CoreRecord> core_stats_;
    std::unordered_set<uint16_t> active_frame_ids_;
    uint64_t sample_seq_ = 0;
    bool sample_section_written_ = false;
    std::chrono::steady_clock::time_point monitor_start_time_;
};

EnhanceMode parseEnhanceMode(const std::string& name) {
    if (name == "tagc") return EnhanceMode::Tagc;
    if (name == "clahe") return EnhanceMode::Clahe;
    if (name == "lut") return EnhanceMode::Lut;
    if (name == "none") return EnhanceMode::None;
    return EnhanceMode::Tagc;
}

void drawDsFusionTargets(cv::Mat& frame_bgr, const std::vector<DsFusionTarget>& ds_targets) {
    if (frame_bgr.empty()) {
        return;
    }

    auto colorBySourceType = [](int source_type) -> cv::Scalar {
        switch (source_type) {
            case 1: return cv::Scalar(0, 255, 0);   // fused
            case 2: return cv::Scalar(0, 165, 255); // radar-only
            case 3: return cv::Scalar(255, 0, 0);   // vision-only
            default: return cv::Scalar(180, 180, 180);
        }
    };

    for (const auto& target : ds_targets) {
        const int x1 = std::max(0, static_cast<int>(target.x));
        const int y1 = std::max(0, static_cast<int>(target.y));
        const int x2 = std::min(frame_bgr.cols - 1, static_cast<int>(target.x + target.w));
        const int y2 = std::min(frame_bgr.rows - 1, static_cast<int>(target.y + target.h));
        if (x2 <= x1 || y2 <= y1) {
            continue;
        }

        const cv::Scalar color = colorBySourceType(target.source_type);
        cv::rectangle(frame_bgr, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);
        std::ostringstream label;
        label << "ds:" << std::fixed << std::setprecision(2) << target.confidence;
        cv::putText(frame_bgr,
                    label.str(),
                    cv::Point(x1, std::max(0, y1 - 6)),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.45,
                    color,
                    1,
                    cv::LINE_AA);
    }
}
}

template <typename T>
class BlockingQueue {
public:
    explicit BlockingQueue(std::size_t max_size = std::numeric_limits<std::size_t>::max())
        : max_size_(max_size) {}

    void push(T value) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (queue_.size() >= max_size_) {
                queue_.pop();
                ++dropped_count_;
            }
            queue_.push(std::move(value));
        }
        cv_.notify_one();
    }

    bool pop(T& out, const std::atomic<bool>& stop_flag) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [&]() { return !queue_.empty() || stop_flag.load(); });
        if (queue_.empty()) {
            return false;
        }
        out = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    void notifyAll() {
        cv_.notify_all();
    }

    std::size_t droppedCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return dropped_count_;
    }

private:
    std::queue<T> queue_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    const std::size_t max_size_;
    std::size_t dropped_count_ = 0;
};

struct FusionStageInput {
    uint16_t frame_id = 0;
    cv::Mat frame_bgr;
    std::vector<DetectionBox> detections;
    std::vector<RadarCluster> clusters;
};

struct DsStageInput {
    uint16_t frame_id = 0;
    cv::Mat frame_bgr;
    std::vector<DetectionBox> detections;
    std::vector<RadarCluster> clusters;
    ChaparisFusionInput fusion_input;
};

struct VisualizationTask {
    uint16_t frame_id = 0;
    cv::Mat frame_bgr;
    std::vector<DsFusionTarget> ds_targets;
};

int main(int argc, char** argv) {
    bindCurrentThreadToCore(THREAD_MAIN_CORE_ID, "main scheduler");

    int target_save_frames = SAVE_FRAMES;
    EnhanceMode enhance_mode = EnhanceMode::Tagc;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-n" || arg == "--frames") && i + 1 < argc) {
            try {
                int parsed = std::stoi(argv[++i]);
                if (parsed > 0) {
                    target_save_frames = parsed;
                }
            } catch (...) {
                std::cerr << "[WARN] invalid frames argument, fallback to default: " << SAVE_FRAMES << "\n";
            }
        } else if (arg == "--enhance" && i + 1 < argc) {
            std::string mode = argv[++i];
            enhance_mode = parseEnhanceMode(mode);
        }
    }

    int ret = system(("mkdir -p " + SAVE_DIR).c_str());
    if (ret != 0) {
        std::cerr << "警告：创建目录失败，可能会影响文件保存\n";
    }
    runtime_log::setLogFile(SAVE_DIR + "/fusion_runtime.log");
    std::cout << "UDP接收器启动，端口: " << UDP_PORT << "\n";
    std::cout << "保存前 " << target_save_frames << " 帧到目录: " << SAVE_DIR << "\n";
    std::cout << "增强模式: " << enhanceModeName(enhance_mode) << "\n";
    std::cout << "从第 " << START_SAVE_FRAME << " 帧开始保存\n\n";
    int saved_frames = 0;
    TimingStats stats;
    std::mutex stats_mutex;
    FrameProcessor processor(stats, saved_frames, stats_mutex, target_save_frames, enhance_mode);

    std::atomic<bool> stop_requested{false};
    RuntimeResourceMonitor resource_monitor(SAVE_DIR + "/resource_usage.log");
    resource_monitor.registerThread("main scheduler", currentThreadTid(), THREAD_MAIN_CORE_ID);
    resource_monitor.start(stop_requested);

    BlockingQueue<std::shared_ptr<CompleteFrame>> b_input_queue(QUEUE_CAP_B_INPUT);
    BlockingQueue<std::shared_ptr<CompleteFrame>> c_input_queue(QUEUE_CAP_C_INPUT);
    BlockingQueue<VisualProcessResult> b_output_queue(QUEUE_CAP_B_OUTPUT);
    BlockingQueue<RadarProcessResult> c_output_queue(QUEUE_CAP_C_OUTPUT);
    BlockingQueue<FusionStageInput> chaparis_queue(QUEUE_CAP_CHAPARIS);
    BlockingQueue<DsStageInput> ds_queue(QUEUE_CAP_DS);
    BlockingQueue<VisualizationTask> visual_queue(QUEUE_CAP_VISUAL);

    std::mutex pending_mutex;
    std::unordered_map<uint16_t, std::shared_ptr<CompleteFrame>> pending_frames;
    std::unordered_map<uint16_t, VisualProcessResult> visual_results;
    std::unordered_map<uint16_t, RadarProcessResult> radar_results;

    std::thread thread_a([&]() {
        bindCurrentThreadToCore(THREAD_A_CORE_ID, "thread A");
        resource_monitor.registerThread("thread A", currentThreadTid(), THREAD_A_CORE_ID);

        int sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) {
            std::cerr << "Failed to create socket\n";
            stop_requested.store(true);
            b_input_queue.notifyAll();
            c_input_queue.notifyAll();
            b_output_queue.notifyAll();
            c_output_queue.notifyAll();
            return;
        }

        int rcvbuf_size = 8 * 1024 * 1024;
        setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf_size, sizeof(rcvbuf_size));
        int reuse_opt = 1;
        setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse_opt, sizeof(reuse_opt));
    #ifdef SO_REUSEPORT
        setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &reuse_opt, sizeof(reuse_opt));
    #endif
        int flags = fcntl(sock, F_GETFL, 0);
        fcntl(sock, F_SETFL, flags | O_NONBLOCK);

        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(UDP_PORT);
        addr.sin_addr.s_addr = INADDR_ANY;
        if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            std::cerr << "Failed to bind socket on UDP " << UDP_PORT
                      << ", errno=" << errno
                      << " (" << std::strerror(errno) << ")\n";
            close(sock);
            stop_requested.store(true);
            b_input_queue.notifyAll();
            c_input_queue.notifyAll();
            b_output_queue.notifyAll();
            c_output_queue.notifyAll();
            return;
        }

        std::unordered_map<uint16_t, CompleteFrame> frame_cache;
        int last_completed_frame_id = -1;

        while (!stop_requested.load()) {
            uint8_t buffer[PACKET_SIZE];
            struct sockaddr_in sender_addr;
            socklen_t sender_len = sizeof(sender_addr);
            ssize_t recv_len = recvfrom(sock, buffer, sizeof(buffer), 0,
                                        (struct sockaddr*)&sender_addr, &sender_len);
            if (recv_len < 0) {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                continue;
            }
            if (recv_len != PACKET_SIZE) {
                continue;
            }

            PacketHeader header = parsePacketHeader(buffer);
            if (header.sub_id >= FRAME_SIZE) {
                continue;
            }

            auto now = std::chrono::steady_clock::now();
            auto& frame = frame_cache[header.frame_id];
            if (frame.packet_arrival_times.empty()) {
                frame.packet_arrival_times.resize(FRAME_SIZE);
            }
            if (frame.received_count == 0) {
                frame.frame_id = header.frame_id;
                frame.timestamp = header.timestamp;
                frame.first_packet_time = now;
            }

            if (header.sub_id < static_cast<uint16_t>(frame.packets.size()) && frame.packets[header.sub_id].empty()) {
                frame.packets[header.sub_id] = std::vector<uint8_t>(buffer + 13, buffer + recv_len);
                frame.packet_arrival_times[header.sub_id] = now;
                frame.received_count++;
            }

            if (frame.received_count == FRAME_SIZE) {
                frame.complete_time = now;
                if (last_completed_frame_id >= 0 &&
                    header.frame_id > static_cast<uint16_t>(last_completed_frame_id + 1)) {
                    std::cout << "[WARN] frame gap detected: last=" << last_completed_frame_id
                              << " current=" << header.frame_id
                              << " missing=" << (header.frame_id - static_cast<uint16_t>(last_completed_frame_id + 1))
                              << "\n";
                }
                last_completed_frame_id = header.frame_id;

                auto frame_ptr = std::make_shared<CompleteFrame>(std::move(frame));
                resource_monitor.onFrameWindowStart(frame_ptr->frame_id);

                {
                    std::vector<uint16_t> dropped_frame_ids;
                    std::lock_guard<std::mutex> lock(pending_mutex);
                    pending_frames[frame_ptr->frame_id] = frame_ptr;
                    while (pending_frames.size() > FRAME_MAP_CAP) {
                        uint16_t dropped_frame_id = 0;
                        if (!popOldestFrameId(pending_frames, dropped_frame_id)) {
                            break;
                        }
                        visual_results.erase(dropped_frame_id);
                        radar_results.erase(dropped_frame_id);
                        dropped_frame_ids.push_back(dropped_frame_id);
                    }
                    for (uint16_t dropped_frame_id : dropped_frame_ids) {
                        resource_monitor.onFrameWindowEnd(dropped_frame_id);
                    }
                }

                b_input_queue.push(frame_ptr);
                c_input_queue.push(frame_ptr);
                frame_cache.erase(header.frame_id);

                if (frame_cache.size() > 32) {
                    auto it = frame_cache.begin();
                    int to_remove = static_cast<int>(frame_cache.size()) - 32;
                    for (int i = 0; i < to_remove; ++i) {
                        std::cout << "[WARN] drop incomplete frame=" << it->first
                                  << " recv=" << it->second.received_count
                                  << "/" << FRAME_SIZE << "\n";
                        it = frame_cache.erase(it);
                    }
                }
            }
        }

        close(sock);
    });

    std::vector<std::thread> thread_b_workers;
    thread_b_workers.reserve(THREAD_B_WORKER_COUNT);
    for (int worker_id = 0; worker_id < THREAD_B_WORKER_COUNT; ++worker_id) {
        thread_b_workers.emplace_back([&, worker_id]() {
            const int bind_core = THREAD_B_CORE_START_ID + worker_id;
            bindCurrentThreadToCore(bind_core, "thread B");
            resource_monitor.registerThread("thread B-" + std::to_string(worker_id), currentThreadTid(), bind_core);
            processor.ensureRknnInitialized();
            while (!stop_requested.load()) {
                std::shared_ptr<CompleteFrame> frame_ptr;
                if (!b_input_queue.pop(frame_ptr, stop_requested)) {
                    continue;
                }
                if (!frame_ptr) {
                    continue;
                }
                auto visual = processor.processVisualBranch(*frame_ptr);
                b_output_queue.push(std::move(visual));
            }
        });
    }

    std::thread thread_c([&]() {
        bindCurrentThreadToCore(THREAD_C_CORE_ID, "thread C");
        resource_monitor.registerThread("thread C", currentThreadTid(), THREAD_C_CORE_ID);
        while (!stop_requested.load()) {
            std::shared_ptr<CompleteFrame> frame_ptr;
            if (!c_input_queue.pop(frame_ptr, stop_requested)) {
                continue;
            }
            if (!frame_ptr) {
                continue;
            }
            auto radar = processor.processRadarBranch(*frame_ptr);
            c_output_queue.push(std::move(radar));
        }
    });

    std::thread thread_chaparis([&]() {
        bindCurrentThreadToCore(THREAD_CHAPARIS_CORE_ID, "chaparis worker");
        resource_monitor.registerThread("chaparis worker", currentThreadTid(), THREAD_CHAPARIS_CORE_ID);
        (void)processor.runChaparisStage({}, {});
        bool skip_first_chaparis_timing = true;
        while (!stop_requested.load()) {
            FusionStageInput in;
            if (!chaparis_queue.pop(in, stop_requested)) {
                continue;
            }
            auto chaparis_start = std::chrono::high_resolution_clock::now();
            DsStageInput ds_in;
            ds_in.frame_id = in.frame_id;
            ds_in.frame_bgr = std::move(in.frame_bgr);
            ds_in.detections = std::move(in.detections);
            ds_in.clusters = std::move(in.clusters);
            ds_in.fusion_input = processor.runChaparisStage(ds_in.detections, ds_in.clusters);
            auto chaparis_end = std::chrono::high_resolution_clock::now();
            double chaparis_time_ms = std::chrono::duration<double, std::milli>(chaparis_end - chaparis_start).count();

            {
                std::lock_guard<std::mutex> lock(stats_mutex);
                if (skip_first_chaparis_timing) {
                    skip_first_chaparis_timing = false;
                } else {
                    stats.total_chaparis_time += chaparis_time_ms;
                }
            }

            const bool fusion_ok = processor.exportFusionInput(ds_in.frame_id, ds_in.fusion_input);
            std::ostringstream fusion_msg;
            fusion_msg << "[F" << ds_in.frame_id << "] fusion_saved=" << (fusion_ok ? 1 : 0)
                       << " matches=" << ds_in.fusion_input.matches.size()
                       << " uR=" << ds_in.fusion_input.uR.size()
                       << " uC=" << ds_in.fusion_input.uC.size();
            runtime_log::logLineFileOnly(fusion_msg.str());

            ds_queue.push(std::move(ds_in));
        }
    });

    std::thread thread_ds([&]() {
        bindCurrentThreadToCore(THREAD_DSFUSION_CORE_ID, "ds worker");
        resource_monitor.registerThread("ds worker", currentThreadTid(), THREAD_DSFUSION_CORE_ID);
        (void)processor.runDsStage(ChaparisFusionInput{}, {}, {});
        bool skip_first_ds_timing = true;
        while (!stop_requested.load()) {
            DsStageInput in;
            if (!ds_queue.pop(in, stop_requested)) {
                continue;
            }
            auto ds_start = std::chrono::high_resolution_clock::now();
            auto ds_result = processor.runDsStage(in.fusion_input, in.detections, in.clusters);
            auto ds_end = std::chrono::high_resolution_clock::now();
            double ds_time_ms = std::chrono::duration<double, std::milli>(ds_end - ds_start).count();

            {
                std::lock_guard<std::mutex> lock(stats_mutex);
                if (skip_first_ds_timing) {
                    skip_first_ds_timing = false;
                } else {
                    stats.total_ds_time += ds_time_ms;
                }
            }

            const bool ds_ok = processor.exportDsResult(in.frame_id, ds_result);
                 std::ostringstream ds_msg;
                 ds_msg << "[F" << in.frame_id << "] ds_saved=" << (ds_ok ? 1 : 0)
                     << " ds_count=" << ds_result.size();
                 runtime_log::logLineFileOnly(ds_msg.str());

            if (!in.frame_bgr.empty()) {
                VisualizationTask task;
                task.frame_id = in.frame_id;
                task.frame_bgr = std::move(in.frame_bgr);
                task.ds_targets = std::move(ds_result);
                visual_queue.push(std::move(task));
            } else {
                resource_monitor.onFrameWindowEnd(in.frame_id);
            }
        }
    });

    std::thread thread_visual([&]() {
        bindCurrentThreadToCore(THREAD_VIS_CORE_ID, "visualize worker");
        resource_monitor.registerThread("visualize worker", currentThreadTid(), THREAD_VIS_CORE_ID);
        const std::string window_name = "DS Fusion Visualization";
        const std::string video_path = SAVE_DIR + "/ds_visualization.avi";
        bool window_ready = true;
        cv::VideoWriter video_writer;
        bool video_writer_ready = false;

        try {
            const int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
            video_writer_ready = video_writer.open(video_path, fourcc, 20.0, cv::Size(IMG_WIDTH, IMG_HEIGHT), true);
            if (!video_writer_ready) {
                std::cerr << "[WARN] cannot open visualization video writer: " << video_path << "\n";
            } else {
                std::cout << "[INFO] visualization video saving: " << video_path << "\n";
            }
        } catch (const std::exception& e) {
            std::cerr << "[WARN] failed to prepare visualization video output: " << e.what() << "\n";
        }

        try {
            cv::startWindowThread();
            cv::namedWindow(window_name, cv::WINDOW_NORMAL);
            cv::Mat warmup_frame(IMG_HEIGHT, IMG_WIDTH, CV_8UC3, cv::Scalar(20, 20, 20));
            cv::putText(warmup_frame,
                        "DS visualization warming up...",
                        cv::Point(30, IMG_HEIGHT / 2),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.9,
                        cv::Scalar(0, 255, 255),
                        2,
                        cv::LINE_AA);
            cv::imshow(window_name, warmup_frame);
            cv::waitKey(1);
        } catch (const cv::Exception& e) {
            window_ready = false;
            std::cerr << "[WARN] visualization window unavailable: " << e.what() << "\n";
        }
        while (!stop_requested.load()) {
            VisualizationTask task;
            if (!visual_queue.pop(task, stop_requested)) {
                continue;
            }
            if (task.frame_bgr.empty()) {
                continue;
            }

            drawDsFusionTargets(task.frame_bgr, task.ds_targets);
            cv::putText(task.frame_bgr,
                        "frame: " + std::to_string(task.frame_id),
                        cv::Point(10, 24),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.7,
                        cv::Scalar(0, 255, 255),
                        2,
                        cv::LINE_AA);
            if (video_writer_ready) {
                video_writer.write(task.frame_bgr);
            }
            if (window_ready) {
                cv::imshow(window_name, task.frame_bgr);
                cv::waitKey(1);
            }
            resource_monitor.onFrameWindowEnd(task.frame_id);
        }
        if (window_ready) {
            cv::destroyWindow(window_name);
        }
        if (video_writer_ready) {
            video_writer.release();
            std::cout << "[INFO] visualization video saved: " << video_path << "\n";
        }
    });

    std::cout << "开始接收数据...\n";
    while (saved_frames < target_save_frames) {
        VisualProcessResult visual;
        if (b_output_queue.pop(visual, stop_requested)) {
            std::lock_guard<std::mutex> lock(pending_mutex);
            visual_results[visual.frame_id] = std::move(visual);
            trimFrameMapByOldestKey(visual_results, FRAME_MAP_CAP);
        }

        RadarProcessResult radar;
        if (c_output_queue.pop(radar, stop_requested)) {
            std::lock_guard<std::mutex> lock(pending_mutex);
            radar_results[radar.frame_id] = std::move(radar);
            trimFrameMapByOldestKey(radar_results, FRAME_MAP_CAP);
        }

        std::vector<uint16_t> ready_ids;
        {
            std::lock_guard<std::mutex> lock(pending_mutex);
            for (const auto& kv : pending_frames) {
                uint16_t frame_id = kv.first;
                if (visual_results.find(frame_id) != visual_results.end() &&
                    radar_results.find(frame_id) != radar_results.end()) {
                    ready_ids.push_back(frame_id);
                }
            }
        }

        for (uint16_t frame_id : ready_ids) {
            std::shared_ptr<CompleteFrame> frame_ptr;
            VisualProcessResult visual_result;
            RadarProcessResult radar_result;
            {
                std::lock_guard<std::mutex> lock(pending_mutex);
                auto frame_it = pending_frames.find(frame_id);
                auto visual_it = visual_results.find(frame_id);
                auto radar_it = radar_results.find(frame_id);
                if (frame_it == pending_frames.end() ||
                    visual_it == visual_results.end() ||
                    radar_it == radar_results.end()) {
                    continue;
                }
                frame_ptr = frame_it->second;
                visual_result = std::move(visual_it->second);
                radar_result = std::move(radar_it->second);
                pending_frames.erase(frame_it);
                visual_results.erase(visual_it);
                radar_results.erase(radar_it);
            }

            if (frame_ptr) {
                FusionStageInput fusion_in;
                fusion_in.frame_id = frame_id;
                fusion_in.frame_bgr = std::move(visual_result.frame_bgr);
                fusion_in.detections = visual_result.detections;
                fusion_in.clusters = radar_result.radar.clusters;
                chaparis_queue.push(std::move(fusion_in));

                processor.finalizeFrame(*frame_ptr, visual_result, radar_result);
            }

            if (saved_frames >= target_save_frames) {
                break;
            }
        }
    }

    stop_requested.store(true);
    b_input_queue.notifyAll();
    c_input_queue.notifyAll();
    b_output_queue.notifyAll();
    c_output_queue.notifyAll();
    chaparis_queue.notifyAll();
    ds_queue.notifyAll();
    visual_queue.notifyAll();

    if (thread_a.joinable()) {
        thread_a.join();
    }
    for (auto& worker : thread_b_workers) {
        if (worker.joinable()) {
            worker.join();
        }
    }
    if (thread_c.joinable()) {
        thread_c.join();
    }
    if (thread_chaparis.joinable()) {
        thread_chaparis.join();
    }
    if (thread_ds.joinable()) {
        thread_ds.join();
    }
    if (thread_visual.joinable()) {
        thread_visual.join();
    }

    resource_monitor.stopAndDump();

    stats.print();
    std::cout << "\n程序结束\n";
    return 0;
}

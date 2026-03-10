#include "rknn_detector.h"
#include "config.h"
#include "runtime_log.h"

#include <opencv2/imgproc.hpp>

#ifdef ENABLE_RKNN_SOURCE_INTEGRATION
#include <cstring>
#include <vector>
#include <filesystem>
#include <mutex>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <chrono>
#include <unistd.h>
#include <fcntl.h>
#include "yolov5.h"
#include "postprocess.h"

struct RknnDetector::Impl {
    mutable std::mutex mutex;
    mutable bool initialized = false;
    mutable bool use_zero_copy = false;
    rknn_app_context_t app_ctx;

    Impl() {
        std::memset(&app_ctx, 0, sizeof(app_ctx));
    }
};

static std::vector<DetectionBox> detect_impl(const rknn_app_context_t* app_ctx,
                                             bool use_zero_copy,
                                             const cv::Mat& image,
                                             float min_score,
                                             bool input_is_rgb) {
    std::vector<DetectionBox> result;
    if (!app_ctx || image.empty()) {
        return result;
    }

    if (image.type() != CV_8UC3) {
        return result;
    }

    cv::Mat input_contiguous = image.isContinuous() ? image : image.clone();
    cv::Mat rgb_image;
    if (input_is_rgb) {
        rgb_image = input_contiguous;
    } else {
        cv::cvtColor(input_contiguous, rgb_image, cv::COLOR_BGR2RGB);
    }

    image_buffer_t src_image;
    std::memset(&src_image, 0, sizeof(src_image));
    src_image.width = rgb_image.cols;
    src_image.height = rgb_image.rows;
    src_image.format = IMAGE_FORMAT_RGB888;
    src_image.virt_addr = reinterpret_cast<unsigned char*>(rgb_image.data);

    object_detect_result_list od_results;
    std::memset(&od_results, 0, sizeof(od_results));

    // 静音底层RGA/letterbox库在stdout上的冗余日志，仅作用于推理调用区间。
    static std::mutex stdout_silence_mutex;
    std::lock_guard<std::mutex> silence_lock(stdout_silence_mutex);
    int old_stdout_fd = dup(STDOUT_FILENO);
    int devnull_fd = open("/dev/null", O_WRONLY);
    if (old_stdout_fd >= 0 && devnull_fd >= 0) {
        dup2(devnull_fd, STDOUT_FILENO);
    }

    rknn_app_context_t* mutable_ctx = const_cast<rknn_app_context_t*>(app_ctx);
    int infer_ret = use_zero_copy
        ? inference_yolov5_zero_copy_model(mutable_ctx, &src_image, &od_results)
        : inference_yolov5_model(mutable_ctx, &src_image, &od_results);

    if (old_stdout_fd >= 0) {
        dup2(old_stdout_fd, STDOUT_FILENO);
        close(old_stdout_fd);
    }
    if (devnull_fd >= 0) {
        close(devnull_fd);
    }

    if (infer_ret != 0) {
        return result;
    }

    auto decode_loop_start = std::chrono::steady_clock::now();
    result.reserve(od_results.count);
    for (int i = 0; i < od_results.count; ++i) {
        const auto& det = od_results.results[i];
        if (det.prop < min_score) {
            continue;
        }

        DetectionBox box;
        box.cls_id = det.cls_id;
        box.score = det.prop;
        const char* cls_name = coco_cls_to_name(det.cls_id);
        box.label = cls_name ? std::string(cls_name) : std::string("unknown");

        int left = std::max(0, det.box.left);
        int top = std::max(0, det.box.top);
        int right = std::min(image.cols - 1, det.box.right);
        int bottom = std::min(image.rows - 1, det.box.bottom);
        if (right <= left || bottom <= top) {
            continue;
        }
        box.box = cv::Rect(left, top, right - left, bottom - top);
        result.push_back(box);
    }
    auto decode_end = std::chrono::steady_clock::now();

    yolov5_perf_t perf = get_last_yolov5_perf();
    double decode_ms = std::chrono::duration<double, std::milli>(decode_end - decode_loop_start).count();
    double total_ms = perf.preprocess_ms + perf.run_ms + perf.postprocess_ms + decode_ms;
    std::ostringstream perf_msg;
    perf_msg << std::fixed << std::setprecision(2)
             << "[RKNN PERF] pre=" << perf.preprocess_ms
             << "ms run=" << perf.run_ms
             << "ms post=" << perf.postprocess_ms
             << "ms decode=" << decode_ms
             << "ms total=" << total_ms << "ms";
    runtime_log::logLineFileOnly(perf_msg.str());
    return result;
}

RknnDetector::RknnDetector() = default;

RknnDetector::~RknnDetector() {
    std::lock_guard<std::mutex> init_lock(init_mutex_);
    for (auto* impl : impls_) {
        if (!impl) {
            continue;
        }
        std::lock_guard<std::mutex> lock(impl->mutex);
        if (impl->initialized) {
            if (impl->use_zero_copy) {
                release_yolov5_zero_copy_model(&impl->app_ctx);
            } else {
                release_yolov5_model(&impl->app_ctx);
            }
            impl->initialized = false;
        }
        delete impl;
    }
    impls_.clear();
    if (initialized_) {
        deinit_post_process();
        initialized_ = false;
    }
}

bool RknnDetector::isAvailable() const {
    return true;
}

bool RknnDetector::init(const std::string& model_path) {
    std::lock_guard<std::mutex> init_lock(init_mutex_);
    if (initialized_) {
        return true;
    }

    std::vector<std::string> candidates;
    if (!model_path.empty()) {
        candidates.push_back(model_path);
    }
    candidates.push_back("rk3588_linux/model/best.rknn");
    candidates.push_back("model/best.rknn");

    int post_ret = init_post_process();
    if (post_ret != 0) {
        return false;
    }

    const int instance_count = std::max(1, RKNN_INSTANCE_COUNT);
    impls_.reserve(instance_count);

    for (int idx = 0; idx < instance_count; ++idx) {
        Impl* impl = new Impl();
        bool model_ok = false;
        bool zero_copy_ok = false;

        for (const auto& candidate : candidates) {
            if (!std::filesystem::exists(candidate)) {
                continue;
            }
            if (init_yolov5_zero_copy_model(candidate.c_str(), &impl->app_ctx) == 0) {
                model_ok = true;
                zero_copy_ok = true;
                break;
            }

            std::memset(&impl->app_ctx, 0, sizeof(impl->app_ctx));
            if (init_yolov5_model(candidate.c_str(), &impl->app_ctx) == 0) {
                model_ok = true;
                break;
            }

            std::memset(&impl->app_ctx, 0, sizeof(impl->app_ctx));
        }

        if (!model_ok) {
            delete impl;
            for (auto* inited : impls_) {
                if (!inited) {
                    continue;
                }
                if (inited->use_zero_copy) {
                    release_yolov5_zero_copy_model(&inited->app_ctx);
                } else {
                    release_yolov5_model(&inited->app_ctx);
                }
                delete inited;
            }
            impls_.clear();
            deinit_post_process();
            return false;
        }

        impl->use_zero_copy = zero_copy_ok;
        impl->initialized = true;

#ifdef RKNN_NPU_CORE_0
        const rknn_core_mask core_masks[] = {
            RKNN_NPU_CORE_0,
            RKNN_NPU_CORE_1,
            RKNN_NPU_CORE_2,
        };
        int mask_ret = rknn_set_core_mask(impl->app_ctx.rknn_ctx, core_masks[idx % 3]);
        if (mask_ret != RKNN_SUCC) {
            std::cerr << "[WARN] rknn_set_core_mask failed for instance " << idx
                      << ", ret=" << mask_ret << "\n";
        }
#endif

        impls_.push_back(impl);
    }

    if (impls_.empty()) {
        deinit_post_process();
        return false;
    }

    initialized_ = true;
    return true;
}

std::vector<DetectionBox> RknnDetector::detect(const cv::Mat& bgr_image, float min_score) const {
    if (!initialized_ || impls_.empty()) {
        return {};
    }
    const size_t idx = rr_counter_.fetch_add(1) % impls_.size();
    Impl* impl = impls_[idx];
    std::lock_guard<std::mutex> lock(impl->mutex);
    if (!impl->initialized) {
        return {};
    }
    return detect_impl(&impl->app_ctx, impl->use_zero_copy, bgr_image, min_score, false);
}

std::vector<DetectionBox> RknnDetector::detectRgb(const cv::Mat& rgb_image, float min_score) const {
    if (!initialized_ || impls_.empty()) {
        return {};
    }
    const size_t idx = rr_counter_.fetch_add(1) % impls_.size();
    Impl* impl = impls_[idx];
    std::lock_guard<std::mutex> lock(impl->mutex);
    if (!impl->initialized) {
        return {};
    }
    return detect_impl(&impl->app_ctx, impl->use_zero_copy, rgb_image, min_score, true);
}

void RknnDetector::drawDetections(cv::Mat& image, const std::vector<DetectionBox>& detections) const {
    for (const auto& det : detections) {
        cv::rectangle(image, det.box, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        const std::string text = cv::format("%s %.1f%%", det.label.c_str(), det.score * 100.0f);
        cv::putText(image, text, cv::Point(det.box.x, std::max(0, det.box.y - 6)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }
}

#else
RknnDetector::RknnDetector() : initialized_(false) {}
RknnDetector::~RknnDetector() = default;

bool RknnDetector::isAvailable() const {
    return false;
}

bool RknnDetector::init(const std::string&) {
    initialized_ = false;
    return false;
}

std::vector<DetectionBox> RknnDetector::detect(const cv::Mat&, float) const {
    return {};
}

std::vector<DetectionBox> RknnDetector::detectRgb(const cv::Mat&, float) const {
    return {};
}

void RknnDetector::drawDetections(cv::Mat&, const std::vector<DetectionBox>&) const {}
#endif

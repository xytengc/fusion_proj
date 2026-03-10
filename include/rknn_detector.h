#ifndef RKNN_DETECTOR_H
#define RKNN_DETECTOR_H

#include <string>
#include <vector>
#include <cstddef>
#include <atomic>
#include <mutex>
#include <opencv2/core.hpp>

struct DetectionBox {
    int cls_id = -1;
    float score = 0.0f;
    cv::Rect box;
    std::string label;
};

class RknnDetector {
public:
    RknnDetector();
    ~RknnDetector();

    bool isAvailable() const;
    bool init(const std::string& model_path);
    std::vector<DetectionBox> detect(const cv::Mat& bgr_image, float min_score) const;
    std::vector<DetectionBox> detectRgb(const cv::Mat& rgb_image, float min_score) const;
    void drawDetections(cv::Mat& image, const std::vector<DetectionBox>& detections) const;

private:
#ifdef ENABLE_RKNN_SOURCE_INTEGRATION
    struct Impl;
    std::vector<Impl*> impls_;
    mutable std::atomic<size_t> rr_counter_{0};
    mutable std::mutex init_mutex_;
    bool initialized_ = false;
#else
    bool initialized_;
#endif
};

#endif // RKNN_DETECTOR_H

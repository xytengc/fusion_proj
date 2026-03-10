#ifndef FRAME_PROCESSOR_H
#define FRAME_PROCESSOR_H


#include "data_recv.h"
#include "tagc_enhancement.h"
#include "config.h"
#include "radar_projection.h"
#include "rknn_detector.h"
#include "chaparis_matcher.h"
#include "ds_fusion.h"
#include <chrono>
#include <atomic>
#include <mutex>
#include <string>
#include <vector>

enum class EnhanceMode {
    Tagc,
    Clahe,
    Lut,
    None
};

const char* enhanceModeName(EnhanceMode mode);

struct TimingStats {
    int total_frames = 0;
    double total_assembly_time = 0;
    double total_tagc_time = 0;
    double total_rknn_time = 0;
    double total_chaparis_time = 0;
    double total_ds_time = 0;
    double total_save_time = 0;
    double total_receive_time = 0;
    int total_receive_samples = 0;
    void print();
};

struct VisualProcessResult {
    uint16_t frame_id = 0;
    cv::Mat frame_bgr;
    cv::Mat detection_only_bgr;
    std::vector<DetectionBox> detections;
    double tagc_time_ms = 0.0;
    double rknn_time_ms = 0.0;
    bool valid = false;
};

struct RadarProcessResult {
    uint16_t frame_id = 0;
    std::vector<uint8_t> raw_packet;
    RadarProcessingResult radar;
    double radar_time_ms = 0.0;
    bool valid = false;
};

class FrameProcessor {
public:
    FrameProcessor(TimingStats& stats, int& saved_frames, std::mutex& stats_mutex,
                   int target_save_frames, EnhanceMode enhance_mode);
    void ensureRknnInitialized();
    VisualProcessResult processVisualBranch(const CompleteFrame& frame);
    RadarProcessResult processRadarBranch(const CompleteFrame& frame);
    ChaparisFusionInput runChaparisStage(const std::vector<DetectionBox>& detections,
                                         const std::vector<RadarCluster>& clusters);
    std::vector<DsFusionTarget> runDsStage(const ChaparisFusionInput& fusion_input,
                                           const std::vector<DetectionBox>& detections,
                                           const std::vector<RadarCluster>& clusters);
    bool exportFusionInput(uint16_t frame_id, const ChaparisFusionInput& fusion_input) const;
    bool exportDsResult(uint16_t frame_id, const std::vector<DsFusionTarget>& ds_targets) const;
    bool finalizeFrame(const CompleteFrame& frame,
                       const VisualProcessResult& visual,
                       const RadarProcessResult& radar);
private:
    IM_TAGC_Enhancement tagc;
    RadarProjector radar_projector;
    ChaparisMatcher chaparis_matcher;
    DsFusionEngine ds_fusion_engine;
    RknnDetector rknn_detector;
    std::once_flag rknn_init_once;
    std::atomic<bool> rknn_ready{false};
    TimingStats& stats;
    int& saved_frames;
    int target_save_frames;
    EnhanceMode enhance_mode;
    std::mutex& stats_mutex;
    std::vector<uint8_t> img_bytes_buffer;
    bool has_prev_complete_time = false;
    std::chrono::steady_clock::time_point prev_complete_time{};
};

#endif // FRAME_PROCESSOR_H

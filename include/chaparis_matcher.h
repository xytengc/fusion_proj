#ifndef CHAPARIS_MATCHER_H
#define CHAPARIS_MATCHER_H

#include "rknn_detector.h"
#include "radar_projection.h"
#include <string>
#include <vector>

struct ChaparisConfig {
    int img_width = 640;
    int img_height = 480;
    float conf_thresh = 0.7f;

    float alpha = 1.0f;
    float beta = 0.5f;
    float gamma = 0.3f;
    float dmax = 50.0f;

    float bbox_expand_ratio_horiz = 0.3f;
    float bbox_expand_ratio_vert = 0.2f;
    float min_expand_pixels = 25.0f;

    float dist_threshold = 5.0f;
    float vel_threshold = 2.0f;

    float cost_unmatched = 1e6f;
};

struct ChaparisMatch {
    int radar_id = -1;   // 1-based, 与 MATLAB 对齐
    int cam_id = -1;     // 1-based, 与 MATLAB 对齐
    float cost = 0.0f;
    float score = 0.0f;
    std::string match_type; // inside / expanded
};

struct ChaparisRefinedMatch {
    ChaparisMatch base_match;
    bool refined = false;
    float original_distance = 0.0f;
    float original_velocity = 0.0f;
    float refined_distance = 0.0f;
    float refined_velocity = 0.0f;
    int additional_radar_id = -1; // 1-based
};

struct ChaparisOcclusionTarget {
    int cam_id = -1;   // 1-based
    int radar_id = -1; // 1-based
    float distance = 0.0f;
    float velocity = 0.0f;
    float distance_diff = 0.0f;
    float velocity_diff = 0.0f;
};

struct ChaparisFusionInput {
    std::vector<ChaparisMatch> matches;
    std::vector<ChaparisRefinedMatch> refined_matches;
    std::vector<ChaparisOcclusionTarget> occlusion_targets;
    std::vector<int> uR;      // 未匹配雷达索引（1-based）
    std::vector<int> uC;      // 未匹配视觉索引（1-based）
    std::vector<float> qR;    // 雷达质量因子
    std::vector<float> qV;    // 视觉质量因子
};

class ChaparisMatcher {
public:
    explicit ChaparisMatcher(ChaparisConfig config = {});

    ChaparisFusionInput run(const std::vector<DetectionBox>& detections,
                            const std::vector<RadarCluster>& radar_clusters) const;

private:
    struct RadarSample {
        float u = 0.0f;
        float v = 0.0f;
        float distance = 0.0f;
        float angle = 0.0f;
        float velocity = 0.0f;
        float signal = 0.0f;
        float density = 0.0f;
    };

    ChaparisConfig cfg_;
};

#endif

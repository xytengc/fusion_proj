#ifndef DS_FUSION_H
#define DS_FUSION_H

#include "chaparis_matcher.h"
#include "radar_projection.h"
#include "rknn_detector.h"
#include <vector>

struct DsFusionConfig {
    float alpha_R = 0.9f;
    float alpha_V = 0.9f;
    float lambda_R = 0.58f;
    float lambda_V = 0.9f;
    float fusion_thresh = 0.5f;

    int image_width = 640;
    int image_height = 480;
    int radar_margin_px = 5;
    int radar_default_w = 40;
    int radar_default_h = 40;
    float radar_size_decay_m = 30.0f;
    float radar_min_scale = 0.25f;
};

struct DsFusionTarget {
    float x = 0.0f;
    float y = 0.0f;
    float w = 0.0f;
    float h = 0.0f;
    float confidence = 0.0f;
    int source_type = 0; // 1=fused, 2=radar-only, 3=vision-only
};

class DsFusionEngine {
public:
    explicit DsFusionEngine(DsFusionConfig config = {});

    std::vector<DsFusionTarget> run(const ChaparisFusionInput& fusion_input,
                                    const std::vector<DetectionBox>& raw_dets,
                                    const std::vector<RadarCluster>& raw_rads) const;

private:
    DsFusionConfig cfg_;
};

#endif

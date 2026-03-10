#include "ds_fusion.h"

#include <algorithm>

namespace {
float clamp01(float v) {
    return std::max(0.0f, std::min(1.0f, v));
}
}

DsFusionEngine::DsFusionEngine(DsFusionConfig config) : cfg_(std::move(config)) {}

std::vector<DsFusionTarget> DsFusionEngine::run(const ChaparisFusionInput& fusion_input,
                                                const std::vector<DetectionBox>& raw_dets,
                                                const std::vector<RadarCluster>& raw_rads) const {
    std::vector<DsFusionTarget> final_targets;
    final_targets.reserve(fusion_input.matches.size() + fusion_input.uR.size() + fusion_input.uC.size());

    for (const auto& m : fusion_input.matches) {
        const int cam_idx = m.cam_id - 1;
        if (cam_idx < 0 || cam_idx >= static_cast<int>(raw_dets.size())) {
            continue;
        }

        const auto& det = raw_dets[cam_idx];
        const float w = static_cast<float>(det.box.width);
        const float h = static_cast<float>(det.box.height);
        const float x = static_cast<float>(det.box.x);
        const float y = static_cast<float>(det.box.y);

        const float conf_V = clamp01(det.score);
        float conf_R = 0.9f;
        const int radar_idx = m.radar_id - 1;
        if (radar_idx >= 0 && radar_idx < static_cast<int>(fusion_input.qR.size())) {
            conf_R = std::max(conf_R, clamp01(fusion_input.qR[radar_idx]));
        }

        const float m_R_H1 = cfg_.alpha_R * conf_R;
        const float m_V_H1 = cfg_.alpha_V * conf_V;
        const float m_final = clamp01(m_R_H1 + m_V_H1 - (m_R_H1 * m_V_H1));

        final_targets.push_back({x, y, w, h, m_final, 1});
    }

    for (const int rid_1based : fusion_input.uR) {
        const int rid = rid_1based - 1;
        if (rid < 0 || rid >= static_cast<int>(raw_rads.size())) {
            continue;
        }

        const float rx = raw_rads[rid].uv.x;
        const float ry = raw_rads[rid].uv.y;
        if (rx < static_cast<float>(cfg_.radar_margin_px) ||
            rx > static_cast<float>(cfg_.image_width - cfg_.radar_margin_px) ||
            ry < static_cast<float>(cfg_.radar_margin_px) ||
            ry > static_cast<float>(cfg_.image_height - cfg_.radar_margin_px)) {
            continue;
        }

        float raw_conf_R = 0.8f;
        if (rid >= 0 && rid < static_cast<int>(fusion_input.qR.size())) {
            raw_conf_R = clamp01(fusion_input.qR[rid]);
        }

        const float m_final = clamp01(cfg_.lambda_R * (cfg_.alpha_R * raw_conf_R));
        if (m_final > cfg_.fusion_thresh) {
            const float range_m = std::max(0.0f, raw_rads[rid].range_m);
            const float decay_denom = std::max(1e-3f, cfg_.radar_size_decay_m);
            const float range_scale = decay_denom / (decay_denom + range_m);
            const float min_scale = clamp01(cfg_.radar_min_scale);
            const float final_scale = std::max(min_scale, range_scale);

            const float dynamic_w = std::max(1.0f, static_cast<float>(cfg_.radar_default_w) * final_scale);
            const float dynamic_h = std::max(1.0f, static_cast<float>(cfg_.radar_default_h) * final_scale);
            const float box_x = rx - 0.5f * dynamic_w;
            const float box_y = ry - 0.5f * dynamic_h;

            final_targets.push_back({
                box_x,
                box_y,
                dynamic_w,
                dynamic_h,
                m_final,
                2
            });
        }
    }

    for (const int cid_1based : fusion_input.uC) {
        const int cid = cid_1based - 1;
        if (cid < 0 || cid >= static_cast<int>(raw_dets.size())) {
            continue;
        }

        const auto& det = raw_dets[cid];
        const float w = static_cast<float>(det.box.width);
        const float h = static_cast<float>(det.box.height);
        const float x = static_cast<float>(det.box.x);
        const float y = static_cast<float>(det.box.y);

        const float raw_conf_V = clamp01(det.score);
        const float m_final = clamp01(cfg_.lambda_V * (cfg_.alpha_V * raw_conf_V));
        if (m_final > cfg_.fusion_thresh) {
            final_targets.push_back({x, y, w, h, m_final, 3});
        }
    }

    return final_targets;
}

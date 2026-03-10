#include "chaparis_matcher.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <utility>

namespace {

constexpr double kBigCost = 1e12;

double clampd(double value, double low, double high) {
    return std::max(low, std::min(high, value));
}

std::vector<int> hungarianSolve(const std::vector<std::vector<double>>& cost) {
    const int n = static_cast<int>(cost.size());
    std::vector<double> u(n + 1, 0.0), v(n + 1, 0.0);
    std::vector<int> p(n + 1, 0), way(n + 1, 0);

    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<double> minv(n + 1, std::numeric_limits<double>::infinity());
        std::vector<char> used(n + 1, false);

        do {
            used[j0] = true;
            const int i0 = p[j0];
            double delta = std::numeric_limits<double>::infinity();
            int j1 = 0;
            for (int j = 1; j <= n; ++j) {
                if (used[j]) {
                    continue;
                }
                const double cur = cost[i0 - 1][j - 1] - u[i0] - v[j];
                if (cur < minv[j]) {
                    minv[j] = cur;
                    way[j] = j0;
                }
                if (minv[j] < delta) {
                    delta = minv[j];
                    j1 = j;
                }
            }

            for (int j = 0; j <= n; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);

        do {
            const int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    std::vector<int> assignment(n, -1);
    for (int j = 1; j <= n; ++j) {
        if (p[j] > 0) {
            assignment[p[j] - 1] = j - 1;
        }
    }
    return assignment;
}

bool pointInRect(float u, float v, const cv::Rect2f& rect) {
    return u >= rect.x && u <= (rect.x + rect.width) &&
           v >= rect.y && v <= (rect.y + rect.height);
}

} // namespace

ChaparisMatcher::ChaparisMatcher(ChaparisConfig config) : cfg_(std::move(config)) {}

ChaparisFusionInput ChaparisMatcher::run(const std::vector<DetectionBox>& detections,
                                         const std::vector<RadarCluster>& radar_clusters) const {
    ChaparisFusionInput out;

    std::vector<DetectionBox> cam;
    cam.reserve(detections.size());
    for (const auto& det : detections) {
        if (det.score >= cfg_.conf_thresh) {
            cam.push_back(det);
        }
    }

    std::vector<RadarSample> radar;
    radar.reserve(radar_clusters.size());
    for (const auto& c : radar_clusters) {
        RadarSample sample;
        sample.u = c.uv.x;
        sample.v = c.uv.y;
        sample.distance = c.range_m;
        sample.angle = c.azimuth_deg;
        sample.velocity = c.velocity_ms;
        sample.signal = static_cast<float>(clampd(c.signal_norm, 0.0, 1.0));
        sample.density = static_cast<float>(clampd(c.density, 0.0, 1.0));
        radar.push_back(sample);
    }

    out.qR.reserve(radar.size());
    for (const auto& r : radar) {
        out.qR.push_back(static_cast<float>(clampd(r.signal * r.density, 0.0, 1.0)));
    }
    out.qV.reserve(cam.size());
    for (const auto& c : cam) {
        out.qV.push_back(c.score);
    }

    if (cam.empty() && radar.empty()) {
        return out;
    }
    if (cam.empty()) {
        for (size_t i = 0; i < radar.size(); ++i) {
            out.uR.push_back(static_cast<int>(i) + 1);
        }
        return out;
    }
    if (radar.empty()) {
        for (size_t j = 0; j < cam.size(); ++j) {
            out.uC.push_back(static_cast<int>(j) + 1);
        }
        return out;
    }

    const size_t nr = radar.size();
    const size_t nc = cam.size();

    std::vector<cv::Rect2f> expanded_bboxes(nc);
    std::vector<cv::Point2f> bbox_centers(nc);
    std::vector<float> bbox_diags(nc, 1.0f);
    std::vector<float> bbox_size_factors(nc, 0.0f);
    std::vector<float> bbox_confs(nc, 0.0f);

    const float img_diag = std::sqrt(static_cast<float>(cfg_.img_width * cfg_.img_width +
                                                         cfg_.img_height * cfg_.img_height));

    for (size_t j = 0; j < nc; ++j) {
        const auto& box = cam[j].box;
        const float x = static_cast<float>(box.x);
        const float y = static_cast<float>(box.y);
        const float w = static_cast<float>(box.width);
        const float h = static_cast<float>(box.height);

        const float expand_w = std::max(w * cfg_.bbox_expand_ratio_horiz, cfg_.min_expand_pixels);
        const float expand_h = std::max(h * cfg_.bbox_expand_ratio_vert, cfg_.min_expand_pixels);

        const float exp_x = std::max(1.0f, x - expand_w * 0.5f);
        const float exp_y = std::max(1.0f, y - expand_h * 0.5f);
        const float exp_w = std::max(1.0f, std::min(static_cast<float>(cfg_.img_width) - exp_x, w + expand_w));
        const float exp_h = std::max(1.0f, std::min(static_cast<float>(cfg_.img_height) - exp_y, h + expand_h));

        expanded_bboxes[j] = cv::Rect2f(exp_x, exp_y, exp_w, exp_h);
        bbox_centers[j] = cv::Point2f(x + w * 0.5f, y + h * 0.5f);
        bbox_diags[j] = std::max(1.0f, std::sqrt(w * w + h * h));
        bbox_size_factors[j] = bbox_diags[j] / std::max(1.0f, img_diag);
        bbox_confs[j] = cam[j].score;
    }

    std::vector<std::vector<double>> raw_cost(nr, std::vector<double>(nc, kBigCost));
    for (size_t i = 0; i < nr; ++i) {
        for (size_t j = 0; j < nc; ++j) {
            if (!pointInRect(radar[i].u, radar[i].v, expanded_bboxes[j])) {
                continue;
            }

            const auto& box = cam[j].box;
            const float x = static_cast<float>(box.x);
            const float y = static_cast<float>(box.y);
            const float w = static_cast<float>(box.width);
            const float h = static_cast<float>(box.height);

            const float du = radar[i].u - bbox_centers[j].x;
            const float dv = radar[i].v - bbox_centers[j].y;
            const float dist_img = std::sqrt(du * du + dv * dv) / bbox_diags[j];

            const bool in_bbox = pointInRect(radar[i].u, radar[i].v, cv::Rect2f(x, y, w, h));
            const float in_bbox_bonus = in_bbox ? 0.5f : 1.0f;

            const float dist_radar = std::min(radar[i].distance / std::max(cfg_.dmax, 1e-3f), 1.0f);

            double cost = cfg_.alpha * dist_img * in_bbox_bonus +
                          cfg_.beta * dist_radar -
                          cfg_.gamma * bbox_confs[j] +
                          0.1f * bbox_size_factors[j];
            cost = std::max(0.0, cost);
            raw_cost[i][j] = cost;
        }
    }

    const int n = static_cast<int>(nr + nc);
    std::vector<std::vector<double>> padded_cost(n, std::vector<double>(n, kBigCost));

    for (size_t i = 0; i < nr; ++i) {
        for (size_t j = 0; j < nc; ++j) {
            padded_cost[i][j] = raw_cost[i][j];
        }
    }

    for (size_t i = 0; i < nr; ++i) {
        for (size_t k = 0; k < nr; ++k) {
            padded_cost[i][nc + k] = (i == k) ? cfg_.cost_unmatched : kBigCost;
        }
    }

    for (size_t j = 0; j < nc; ++j) {
        for (size_t k = 0; k < nc; ++k) {
            padded_cost[nr + j][k] = (j == k) ? cfg_.cost_unmatched : kBigCost;
        }
    }

    for (size_t i = 0; i < nc; ++i) {
        for (size_t j = 0; j < nr; ++j) {
            padded_cost[nr + i][nc + j] = 0.0;
        }
    }

    const std::vector<int> assignment = hungarianSolve(padded_cost);

    std::set<int> matched_cols;
    for (size_t i = 0; i < nr; ++i) {
        const int col = assignment[i];
        if (col >= 0 && col < static_cast<int>(nc)) {
            const double c = raw_cost[i][col];
            if (std::isfinite(c) && c < static_cast<double>(cfg_.cost_unmatched)) {
                ChaparisMatch match;
                match.radar_id = static_cast<int>(i) + 1;
                match.cam_id = col + 1;
                match.cost = static_cast<float>(c);
                match.score = static_cast<float>(std::max(0.0, 1.0 - c));

                const auto& b = cam[static_cast<size_t>(col)].box;
                const bool inside = pointInRect(radar[i].u, radar[i].v,
                                                cv::Rect2f(static_cast<float>(b.x),
                                                           static_cast<float>(b.y),
                                                           static_cast<float>(b.width),
                                                           static_cast<float>(b.height)));
                match.match_type = inside ? "inside" : "expanded";
                out.matches.push_back(match);
                matched_cols.insert(col);
            } else {
                out.uR.push_back(static_cast<int>(i) + 1);
            }
        } else {
            out.uR.push_back(static_cast<int>(i) + 1);
        }
    }

    for (size_t j = 0; j < nc; ++j) {
        if (matched_cols.find(static_cast<int>(j)) == matched_cols.end()) {
            out.uC.push_back(static_cast<int>(j) + 1);
        }
    }

    struct UnmatchedInBboxPoint {
        int radar_id = -1; // 0-based
        int cam_id = -1;   // 0-based
        float distance = 0.0f;
        float velocity = 0.0f;
    };

    std::vector<UnmatchedInBboxPoint> unmatched_in_bbox;
    unmatched_in_bbox.reserve(out.uR.size());

    for (int radar_id_1based : out.uR) {
        const int radar_id = radar_id_1based - 1;
        if (radar_id < 0 || radar_id >= static_cast<int>(nr)) {
            continue;
        }

        const float u = radar[static_cast<size_t>(radar_id)].u;
        const float v = radar[static_cast<size_t>(radar_id)].v;

        for (size_t cam_id = 0; cam_id < nc; ++cam_id) {
            const auto& box = cam[cam_id].box;
            if (pointInRect(u, v, cv::Rect2f(static_cast<float>(box.x),
                                             static_cast<float>(box.y),
                                             static_cast<float>(box.width),
                                             static_cast<float>(box.height)))) {
                unmatched_in_bbox.push_back({
                    radar_id,
                    static_cast<int>(cam_id),
                    radar[static_cast<size_t>(radar_id)].distance,
                    radar[static_cast<size_t>(radar_id)].velocity,
                });
                break;
            }
        }
    }

    if (!unmatched_in_bbox.empty()) {
        for (size_t cam_id = 0; cam_id < nc; ++cam_id) {
            auto best_it = std::find_if(out.matches.begin(), out.matches.end(),
                                        [cam_id](const ChaparisMatch& m) {
                                            return m.cam_id == static_cast<int>(cam_id) + 1;
                                        });
            if (best_it == out.matches.end()) {
                continue;
            }

            const int best_radar_id = best_it->radar_id - 1;
            if (best_radar_id < 0 || best_radar_id >= static_cast<int>(nr)) {
                continue;
            }

            const float best_distance = radar[static_cast<size_t>(best_radar_id)].distance;
            const float best_velocity = radar[static_cast<size_t>(best_radar_id)].velocity;

            for (const auto& point : unmatched_in_bbox) {
                if (point.cam_id != static_cast<int>(cam_id)) {
                    continue;
                }

                const float dist_diff = std::abs(point.distance - best_distance);
                const float vel_diff = std::abs(point.velocity - best_velocity);

                if (dist_diff < cfg_.dist_threshold && vel_diff < cfg_.vel_threshold) {
                    ChaparisRefinedMatch refined;
                    refined.base_match = *best_it;
                    refined.refined = true;
                    refined.original_distance = best_distance;
                    refined.original_velocity = best_velocity;
                    refined.refined_distance = (best_distance + point.distance) * 0.5f;
                    refined.refined_velocity = (best_velocity + point.velocity) * 0.5f;
                    refined.additional_radar_id = point.radar_id + 1;
                    out.refined_matches.push_back(refined);
                } else {
                    ChaparisOcclusionTarget occlusion;
                    occlusion.cam_id = static_cast<int>(cam_id) + 1;
                    occlusion.radar_id = point.radar_id + 1;
                    occlusion.distance = point.distance;
                    occlusion.velocity = point.velocity;
                    occlusion.distance_diff = dist_diff;
                    occlusion.velocity_diff = vel_diff;
                    out.occlusion_targets.push_back(occlusion);
                }
            }
        }
    }

    return out;
}

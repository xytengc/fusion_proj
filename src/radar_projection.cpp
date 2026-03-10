#include "radar_projection.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>

namespace {
inline uint16_t read_le_u16(const uint8_t* data) {
    return static_cast<uint16_t>(data[0]) |
           static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8);
}

inline int16_t read_le_i16(const uint8_t* data) {
    return static_cast<int16_t>(read_le_u16(data));
}

inline uint32_t read_le_u32(const uint8_t* data) {
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) |
           (static_cast<uint32_t>(data[3]) << 24);
}
}

RadarProjector::RadarProjector() {
    camera_matrix_ = cv::Matx33f(
        CAMERA_INTRINSIC[0], CAMERA_INTRINSIC[1], CAMERA_INTRINSIC[2],
        CAMERA_INTRINSIC[3], CAMERA_INTRINSIC[4], CAMERA_INTRINSIC[5],
        CAMERA_INTRINSIC[6], CAMERA_INTRINSIC[7], CAMERA_INTRINSIC[8]);

    dist_coeffs_ = cv::Vec4f(
        CAMERA_DISTORTION[0], CAMERA_DISTORTION[1],
        CAMERA_DISTORTION[2], CAMERA_DISTORTION[3]);

    radar_to_cam_ = cv::Matx44f(
        RADAR_TO_CAMERA[0],  RADAR_TO_CAMERA[1],  RADAR_TO_CAMERA[2],  RADAR_TO_CAMERA[3],
        RADAR_TO_CAMERA[4],  RADAR_TO_CAMERA[5],  RADAR_TO_CAMERA[6],  RADAR_TO_CAMERA[7],
        RADAR_TO_CAMERA[8],  RADAR_TO_CAMERA[9],  RADAR_TO_CAMERA[10], RADAR_TO_CAMERA[11],
        RADAR_TO_CAMERA[12], RADAR_TO_CAMERA[13], RADAR_TO_CAMERA[14], RADAR_TO_CAMERA[15]);
}

RadarProcessingResult RadarProjector::processPacket(const std::vector<uint8_t>& packet,
                                                    const cv::Size& image_size) const {
    RadarProcessingResult result;
    result.raw_points = decodePacket(packet);

    auto filtered_points = filterPoints(result.raw_points);
    if (filtered_points.empty()) {
        return result;
    }

    auto radar_xyz = toCartesian(filtered_points);
    if (radar_xyz.empty()) {
        return result;
    }

    std::vector<cv::Point2f> xy;
    xy.reserve(radar_xyz.size());
    for (const auto& p : radar_xyz) {
        xy.emplace_back(p.x, p.y);
    }

    auto labels = dbscan(xy);
    auto clusters = buildClusters(filtered_points, radar_xyz, labels);

    std::vector<RadarCluster> projected;
    projected.reserve(clusters.size());
    for (auto cluster : clusters) {
        cv::Point2f uv;
        if (projectToImage(cluster.xyz, image_size, uv)) {
            cluster.uv = uv;
            projected.push_back(cluster);
        }
    }

    // 与 MATLAB projdbscan.m 对齐：Signal 在有效投影聚类后做帧内最大值归一化
    float max_signal = 0.0f;
    for (const auto& cluster : projected) {
        if (cluster.power_dbm > max_signal) {
            max_signal = cluster.power_dbm;
        }
    }
    if (max_signal > 1e-6f) {
        for (auto& cluster : projected) {
            cluster.signal_norm = cluster.power_dbm / max_signal;
        }
    }

    // 与 MATLAB projdbscan.m 对齐：密度在聚类后计算并做归一化
    // cluster_volume = epsilon^2 * N, density = N / cluster_volume
    // 最终 density = density / max(density)
    const float eps2 = std::max(RADAR_DBSCAN_EPSILON * RADAR_DBSCAN_EPSILON, 1e-6f);
    float max_density = 0.0f;
    for (auto& cluster : projected) {
        const float n = static_cast<float>(std::max<size_t>(cluster.size, 1));
        const float cluster_volume = eps2 * n;
        cluster.density = n / std::max(cluster_volume, 1e-6f);
        if (cluster.density > max_density) {
            max_density = cluster.density;
        }
    }
    if (max_density > 1e-6f) {
        for (auto& cluster : projected) {
            cluster.density /= max_density;
        }
    }

    result.clusters = std::move(projected);
    return result;
}

void RadarProjector::drawClusters(cv::Mat& image, const std::vector<RadarCluster>& clusters) const {
    if (clusters.empty()) {
        return;
    }

    for (const auto& cluster : clusters) {
        if (cluster.uv.x < 0 || cluster.uv.y < 0) {
            continue;
        }
        const cv::Scalar color = rangeToColor(cluster.range_m);
        cv::circle(image, cluster.uv, 6, color, 2, cv::LINE_AA);
        const std::string text = cv::format("R%.1f V%.1f", cluster.range_m, cluster.velocity_ms);
        cv::putText(image, text, cluster.uv + cv::Point2f(8.f, -8.f), cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv::LINE_AA);
    }
}

void RadarProjector::exportAkbkTxt(const std::string& filepath, int frame_id,
                                   const std::vector<RadarPoint>& points) const {
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
        return;
    }

    ofs << "--- F " << frame_id << '\n';
    ofs << "AK\n";
    ofs << std::fixed << std::setprecision(2);
    for (const auto& pt : points) {
        ofs << pt.id << ": "
            << "P " << std::setw(7) << pt.power_dbm << ' '
            << "R " << std::setw(7) << pt.range_m << ' '
            << "V " << std::setw(7) << pt.velocity_ms << ' '
            << "A " << std::setw(7) << pt.azimuth_deg << '\n';
    }
    ofs << "BK\n#\n";
}

std::vector<RadarPoint> RadarProjector::decodePacket(const std::vector<uint8_t>& packet) const {
    std::vector<RadarPoint> points;
    if (packet.size() < 4) {
        return points;
    }

    // 与 udp_verifier.py 保持一致：
    // radar_data[:4] 为 little-endian 点数，后续每个点 16 字节，解析格式 <HhhhHHHH
    const uint32_t reported_points = read_le_u32(packet.data());
    const uint32_t max_possible = static_cast<uint32_t>((packet.size() - 4) / 16);
    const uint32_t count = std::min<uint32_t>({reported_points, max_possible, 64u});
    size_t offset = 4;

    points.reserve(count);
    for (uint32_t i = 0; i < count; ++i, offset += 16) {
        if (offset + 16 > packet.size()) {
            break;
        }
        const uint8_t* ptr = packet.data() + offset;
        RadarPoint pt;
        pt.id = read_le_u16(ptr);
        pt.range_m = static_cast<float>(read_le_i16(ptr + 2)) * 0.01f;
        pt.velocity_ms = static_cast<float>(read_le_i16(ptr + 4)) * 0.01f;
        pt.azimuth_deg = static_cast<float>(read_le_i16(ptr + 6)) * 0.01f;
        pt.power_dbm = static_cast<float>(read_le_u16(ptr + 8)) * 0.01f;
        pt.elevation_deg = 0.0f;
        points.push_back(pt);
    }
    return points;
}

std::vector<RadarPoint> RadarProjector::filterPoints(const std::vector<RadarPoint>& points) const {
    std::vector<RadarPoint> filtered;
    filtered.reserve(points.size());
    for (const auto& pt : points) {
        if (pt.range_m < RADAR_MIN_RANGE_M) {
            continue;
        }
        if (std::abs(pt.velocity_ms) < RADAR_MIN_SPEED_MS) {
            continue;
        }
        filtered.push_back(pt);
    }
    return filtered;
}

std::vector<cv::Point3f> RadarProjector::toCartesian(const std::vector<RadarPoint>& points) const {
    std::vector<cv::Point3f> xyz;
    xyz.reserve(points.size());
    constexpr float DEG2RAD = static_cast<float>(CV_PI / 180.0);

    for (const auto& pt : points) {
        const float range = pt.range_m;
        const float azimuth = pt.azimuth_deg * DEG2RAD;
        const float elevation = pt.elevation_deg * DEG2RAD;

        const float cos_e = std::cos(elevation);
        const float x = range * cos_e * std::cos(azimuth);
        const float y = range * cos_e * std::sin(azimuth);
        const float z = RADAR_SCALE_FACTOR * range;
        xyz.emplace_back(x, y, z);
    }
    return xyz;
}

std::vector<int> RadarProjector::dbscan(const std::vector<cv::Point2f>& xy) const {
    const size_t n = xy.size();
    std::vector<int> labels(n, 0);
    std::vector<bool> visited(n, false);
    if (n == 0) {
        return labels;
    }

    // 与 MATLAB projdbscan.m 对齐：先做标准化再在 xy 平面聚类
    std::vector<cv::Point2f> xy_norm = xy;
    float mean_x = 0.0f;
    float mean_y = 0.0f;
    for (const auto& p : xy_norm) {
        mean_x += p.x;
        mean_y += p.y;
    }
    mean_x /= static_cast<float>(n);
    mean_y /= static_cast<float>(n);

    float var_x = 0.0f;
    float var_y = 0.0f;
    for (const auto& p : xy_norm) {
        const float dx = p.x - mean_x;
        const float dy = p.y - mean_y;
        var_x += dx * dx;
        var_y += dy * dy;
    }
    const float std_x = std::sqrt(var_x / static_cast<float>(n)) + 1e-6f;
    const float std_y = std::sqrt(var_y / static_cast<float>(n)) + 1e-6f;

    if (n > 1) {
        for (auto& p : xy_norm) {
            p.x = (p.x - mean_x) / std_x;
            p.y = (p.y - mean_y) / std_y;
        }
    }

    const float eps2 = RADAR_DBSCAN_EPSILON * RADAR_DBSCAN_EPSILON;
    int cluster_id = 0;

    auto regionQuery = [&](size_t idx) {
        std::vector<int> neighbors;
        neighbors.reserve(n);
        for (size_t j = 0; j < n; ++j) {
            const float dx = xy_norm[j].x - xy_norm[idx].x;
            const float dy = xy_norm[j].y - xy_norm[idx].y;
            if ((dx * dx + dy * dy) <= eps2) {
                neighbors.push_back(static_cast<int>(j));
            }
        }
        return neighbors;
    };

    for (size_t i = 0; i < n; ++i) {
        if (visited[i]) {
            continue;
        }
        visited[i] = true;
        auto neighbors = regionQuery(i);
        if (neighbors.size() < static_cast<size_t>(RADAR_DBSCAN_MIN_POINTS)) {
            labels[i] = -1;
            continue;
        }

        ++cluster_id;
        labels[i] = cluster_id;
        std::vector<int> seeds = neighbors;
        for (size_t k = 0; k < seeds.size(); ++k) {
            int idx = seeds[k];
            if (!visited[idx]) {
                visited[idx] = true;
                auto local_neighbors = regionQuery(idx);
                if (local_neighbors.size() >= static_cast<size_t>(RADAR_DBSCAN_MIN_POINTS)) {
                    seeds.insert(seeds.end(), local_neighbors.begin(), local_neighbors.end());
                }
            }
            if (labels[idx] <= 0) {
                labels[idx] = cluster_id;
            }
        }
    }

    return labels;
}

std::vector<RadarCluster> RadarProjector::buildClusters(const std::vector<RadarPoint>& points,
                                                        const std::vector<cv::Point3f>& xyz,
                                                        const std::vector<int>& labels) const {
    std::vector<RadarCluster> clusters;
    if (points.empty() || xyz.empty() || labels.empty()) {
        return clusters;
    }

    const int max_label = *std::max_element(labels.begin(), labels.end());
    if (max_label <= 0) {
        return clusters;
    }

    clusters.reserve(max_label);
    for (int cluster_id = 1; cluster_id <= max_label; ++cluster_id) {
        cv::Point3f xyz_accum(0.f, 0.f, 0.f);
        float r_accum = 0.f;
        float a_accum = 0.f;
        float v_accum = 0.f;
        float p_accum = 0.f;
        float weight_sum = 0.f;
        size_t count = 0;

        for (size_t i = 0; i < points.size(); ++i) {
            if (labels[i] != cluster_id) {
                continue;
            }
            const float weight = 1.0f / std::max(points[i].range_m, 0.1f);
            xyz_accum += xyz[i] * weight;
            r_accum += points[i].range_m * weight;
            a_accum += points[i].azimuth_deg * weight;
            v_accum += points[i].velocity_ms * weight;
            p_accum += points[i].power_dbm * weight;
            weight_sum += weight;
            ++count;
        }

        if (count == 0 || weight_sum <= 0.f) {
            continue;
        }

        RadarCluster cluster;
        cluster.xyz = xyz_accum * (1.0f / weight_sum);
        cluster.range_m = r_accum / weight_sum;
        cluster.azimuth_deg = a_accum / weight_sum;
        cluster.velocity_ms = v_accum / weight_sum;
        cluster.power_dbm = p_accum / weight_sum;
        cluster.signal_norm = 0.0f;
        cluster.density = 0.0f;
        cluster.size = count;
        cluster.uv = cv::Point2f(-1.f, -1.f);
        clusters.push_back(cluster);
    }

    return clusters;
}

bool RadarProjector::projectToImage(const cv::Point3f& radar_xyz, const cv::Size& image_size,
                                    cv::Point2f& uv_out) const {
    cv::Vec4f radar_vec(radar_xyz.x, radar_xyz.y, radar_xyz.z, 1.0f);
    cv::Vec4f cam = radar_to_cam_ * radar_vec;
    if (cam[2] <= 1e-3f) {
        return false;
    }

    float x = cam[0] / cam[2];
    float y = cam[1] / cam[2];
    float r2 = x * x + y * y;
    float radial = 1.0f + dist_coeffs_[0] * r2 + dist_coeffs_[1] * r2 * r2;

    float x_distorted = x * radial + 2.0f * dist_coeffs_[2] * x * y + dist_coeffs_[3] * (r2 + 2.0f * x * x);
    float y_distorted = y * radial + dist_coeffs_[2] * (r2 + 2.0f * y * y) + 2.0f * dist_coeffs_[3] * x * y;

    float u = camera_matrix_(0, 0) * x_distorted + camera_matrix_(0, 2) + RADAR_PROJECTION_U_OFFSET;
    float v = camera_matrix_(1, 1) * y_distorted + camera_matrix_(1, 2);

    if (u < 0 || v < 0 || u >= static_cast<float>(image_size.width) ||
        v >= static_cast<float>(image_size.height)) {
        return false;
    }

    uv_out = cv::Point2f(u, v);
    return true;
}

cv::Scalar RadarProjector::rangeToColor(float range_m) const {
    (void)range_m;
    return cv::Scalar(0, 255, 255);
}

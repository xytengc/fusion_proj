#ifndef RADAR_PROJECTION_H
#define RADAR_PROJECTION_H

#include <vector>
#include <cstdint>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "config.h"

struct RadarPoint {
    uint16_t id = 0;
    float range_m = 0.0f;
    float velocity_ms = 0.0f;
    float azimuth_deg = 0.0f;
    float elevation_deg = 0.0f;
    float power_dbm = 0.0f;
};

struct RadarCluster {
    cv::Point3f xyz;          // 雷达坐标系坐标
    cv::Point2f uv;           // 图像像素坐标
    float range_m = 0.0f;
    float azimuth_deg = 0.0f;
    float velocity_ms = 0.0f;
    float power_dbm = 0.0f;
    float signal_norm = 0.0f;
    float density = 0.0f;
    size_t size = 0;
};

struct RadarProcessingResult {
    std::vector<RadarPoint> raw_points;
    std::vector<RadarCluster> clusters;
};

class RadarProjector {
public:
    RadarProjector();

    RadarProcessingResult processPacket(const std::vector<uint8_t>& packet,
                                        const cv::Size& image_size) const;

    void drawClusters(cv::Mat& image, const std::vector<RadarCluster>& clusters) const;

    void exportAkbkTxt(const std::string& filepath, int frame_id,
                       const std::vector<RadarPoint>& points) const;

private:
    cv::Matx33f camera_matrix_;
    cv::Vec4f dist_coeffs_;
    cv::Matx44f radar_to_cam_;

    std::vector<RadarPoint> decodePacket(const std::vector<uint8_t>& packet) const;
    std::vector<RadarPoint> filterPoints(const std::vector<RadarPoint>& points) const;
    std::vector<cv::Point3f> toCartesian(const std::vector<RadarPoint>& points) const;
    std::vector<int> dbscan(const std::vector<cv::Point2f>& xy) const;
    std::vector<RadarCluster> buildClusters(const std::vector<RadarPoint>& points,
                                            const std::vector<cv::Point3f>& xyz,
                                            const std::vector<int>& labels) const;
    bool projectToImage(const cv::Point3f& radar_xyz, const cv::Size& image_size,
                        cv::Point2f& uv_out) const;
    cv::Scalar rangeToColor(float range_m) const;
};

#endif // RADAR_PROJECTION_H

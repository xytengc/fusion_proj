#include "frame_processor.h"
#include "config.h"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iomanip>
#include <cstring>
#include <chrono>
#include <iostream>
#include <thread>
#include "thread_affinity.h"
#include "runtime_log.h"

namespace {
cv::Mat enhanceWithClahe(const cv::Mat& rgb) {
    cv::Mat ycrcb;
    cv::cvtColor(rgb, ycrcb, cv::COLOR_RGB2YCrCb);
    std::vector<cv::Mat> channels;
    cv::split(ycrcb, channels);

    static thread_local cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
    clahe->apply(channels[0], channels[0]);

    cv::merge(channels, ycrcb);
    cv::Mat out_rgb;
    cv::cvtColor(ycrcb, out_rgb, cv::COLOR_YCrCb2RGB);
    return out_rgb;
}

cv::Mat enhanceWithLutGamma(const cv::Mat& rgb) {
    cv::Mat gray;
    cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
    float mean_norm = static_cast<float>(cv::mean(gray)[0] / 255.0);

    float gamma = 1.0f;
    if (mean_norm < 0.5f) {
        gamma = std::max(0.6f, 1.0f - (0.5f - mean_norm));
    } else {
        gamma = std::min(1.4f, 1.0f + (mean_norm - 0.5f));
    }

    cv::Mat lut(1, 256, CV_8U);
    uchar* ptr = lut.ptr<uchar>();
    for (int i = 0; i < 256; ++i) {
        float x = static_cast<float>(i) / 255.0f;
        ptr[i] = cv::saturate_cast<uchar>(std::pow(x, gamma) * 255.0f + 0.5f);
    }

    cv::Mat out_rgb;
    cv::LUT(rgb, lut, out_rgb);
    return out_rgb;
}

bool exportDetectionsTxt(const std::string& filepath,
                         int frame_id,
                         const std::vector<DetectionBox>& detections) {
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
        return false;
    }

    ofs << "--- F " << frame_id << "\n";
    ofs << "DET_COUNT " << detections.size() << "\n";
    ofs << "# cls_id score x y w h label\n";
    ofs << std::fixed << std::setprecision(4);
    for (const auto& det : detections) {
        ofs << det.cls_id << ' '
            << det.score << ' '
            << det.box.x << ' '
            << det.box.y << ' '
            << det.box.width << ' '
            << det.box.height << ' '
            << det.label << "\n";
    }
    ofs << "#\n";
    return ofs.good();
}

bool exportRadarFusionTxt(const std::string& filepath,
                         int frame_id,
                         const std::vector<RadarPoint>& raw_points,
                         const std::vector<RadarCluster>& clusters) {
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
        return false;
    }

    ofs << "--- F " << frame_id << "\n";
    ofs << "RAW_POINTS\n";
    ofs << std::fixed << std::setprecision(3);
    for (const auto& pt : raw_points) {
        ofs << pt.id << ": "
            << "P " << pt.power_dbm << ' '
            << "R " << pt.range_m << ' '
            << "V " << pt.velocity_ms << ' '
            << "A " << pt.azimuth_deg << ' '
            << "E " << pt.elevation_deg << "\n";
    }
    ofs << "PROJECTED_CLUSTERS\n";
    for (size_t i = 0; i < clusters.size(); ++i) {
        const auto& c = clusters[i];
        ofs << i << ": "
            << "X " << c.xyz.x << ' '
            << "Y " << c.xyz.y << ' '
            << "Z " << c.xyz.z << ' '
            << "U " << c.uv.x << ' '
            << "V " << c.uv.y << ' '
            << "R " << c.range_m << ' '
            << "A " << c.azimuth_deg << ' '
            << "Vel " << c.velocity_ms << ' '
            << "P " << c.power_dbm << ' '
            << "SNorm " << c.signal_norm << ' '
            << "Den " << c.density << ' '
            << "N " << c.size << "\n";
    }
    ofs << "#\n";
    return ofs.good();
}

bool exportFusionInputTxt(const std::string& filepath,
                          int frame_id,
                          const ChaparisFusionInput& fusion_input) {
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
        return false;
    }

    ofs << "--- F " << frame_id << "\n";
    ofs << "MATCHES " << fusion_input.matches.size() << "\n";
    ofs << std::fixed << std::setprecision(6);
    for (const auto& m : fusion_input.matches) {
        ofs << m.radar_id << ' '
            << m.cam_id << ' '
            << m.cost << ' '
            << m.score << ' '
            << m.match_type << "\n";
    }

    ofs << "UR " << fusion_input.uR.size();
    for (int idx : fusion_input.uR) {
        ofs << ' ' << idx;
    }
    ofs << "\n";

    ofs << "UC " << fusion_input.uC.size();
    for (int idx : fusion_input.uC) {
        ofs << ' ' << idx;
    }
    ofs << "\n";

    ofs << "QR " << fusion_input.qR.size();
    for (float q : fusion_input.qR) {
        ofs << ' ' << q;
    }
    ofs << "\n";

    ofs << "QV " << fusion_input.qV.size();
    for (float q : fusion_input.qV) {
        ofs << ' ' << q;
    }
    ofs << "\n#\n";
    return ofs.good();
}

bool exportDsResultTxt(const std::string& filepath,
                       int frame_id,
                       const std::vector<DsFusionTarget>& ds_targets) {
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
        return false;
    }

    ofs << "--- F " << frame_id << "\n";
    ofs << "DS_COUNT " << ds_targets.size() << "\n";
    ofs << "# x y w h confidence source_type\n";
    ofs << std::fixed << std::setprecision(6);
    for (const auto& t : ds_targets) {
        ofs << t.x << ' '
            << t.y << ' '
            << t.w << ' '
            << t.h << ' '
            << t.confidence << ' '
            << t.source_type << "\n";
    }
    ofs << "#\n";
    return ofs.good();
}

const char* modeNameLocal(EnhanceMode mode) {
    switch (mode) {
        case EnhanceMode::Tagc: return "tagc";
        case EnhanceMode::Clahe: return "clahe";
        case EnhanceMode::Lut: return "lut";
        case EnhanceMode::None: return "none";
        default: return "tagc";
    }
}
}

const char* enhanceModeName(EnhanceMode mode) {
    return modeNameLocal(mode);
}

void TimingStats::print() {
    if (total_frames == 0) return;
    const int timed_frames = std::max(0, total_frames - 1);
    double avg_assembly = (timed_frames > 0) ? (total_assembly_time / timed_frames) : 0.0;
    double avg_tagc = (timed_frames > 0) ? (total_tagc_time / timed_frames) : 0.0;
    double avg_rknn = (timed_frames > 0) ? (total_rknn_time / timed_frames) : 0.0;
        double avg_chaparis = (timed_frames > 0) ? (total_chaparis_time / timed_frames) : 0.0;
        double avg_ds = (timed_frames > 0) ? (total_ds_time / timed_frames) : 0.0;
    double avg_save = (timed_frames > 0) ? (total_save_time / timed_frames) : 0.0;
    double avg_receive = (total_receive_samples > 0)
        ? (total_receive_time / total_receive_samples)
        : 0.0;
        double avg_total = avg_assembly + avg_tagc + avg_rknn + avg_chaparis + avg_ds + avg_save;
    std::cout << "平均网络接收帧间隔: " << std::fixed << std::setprecision(2) << avg_receive << " ms\n";
    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << "最终统计信息:\n";
    std::cout << std::string(60, '=') << "\n";
    std::cout << "成功保存帧数: " << total_frames << "\n";
    std::cout << "计时统计帧数(已剔除首帧): " << timed_frames << "\n";
    std::cout << "\n平均每帧处理时间:\n";
    
    std::cout << "  ├─ 图像组装: " << avg_assembly << " ms\n";
    std::cout << "  ├─ TAGC处理: " << avg_tagc << " ms\n";
    std::cout << "  ├─ RKNN推理: " << avg_rknn << " ms\n";
        std::cout << "  ├─ Chaparis匹配: " << avg_chaparis << " ms\n";
        std::cout << "  ├─ DS融合: " << avg_ds << " ms\n";
    std::cout << "  └─ 文件保存: " << avg_save << " ms\n";
    if (avg_total > 0.0) {
        std::cout << "  └─ 总计: " << avg_total << " ms (" << 1000.0/avg_total << " fps)\n";
    } else {
        std::cout << "  └─ 总计: " << avg_total << " ms\n";
    }
}

FrameProcessor::FrameProcessor(TimingStats& s, int& sf, std::mutex& m,
                                                             int target, EnhanceMode mode)
                : chaparis_matcher(ChaparisConfig{}),
                    ds_fusion_engine(DsFusionConfig{}),
                    stats(s), saved_frames(sf), target_save_frames(target), enhance_mode(mode),
            stats_mutex(m), img_bytes_buffer(IMG_WIDTH * IMG_HEIGHT * 3, 0) {
        std::cout << "[INFO] visual enhance mode: " << enhanceModeName(enhance_mode) << "\n";
}

void FrameProcessor::ensureRknnInitialized() {
    if (!ENABLE_RKNN_DETECT || !rknn_detector.isAvailable()) {
        return;
    }

    std::call_once(rknn_init_once, [&]() {
        const bool ok = rknn_detector.init(RKNN_MODEL_PATH);
        rknn_ready.store(ok, std::memory_order_release);
        if (!ok) {
            std::cerr << "[WARN] RKNN detector init failed, fallback to no-detection mode\n";
        } else {
            std::cout << "[INFO] RKNN detector pre-initialized\n";
        }
    });
}

VisualProcessResult FrameProcessor::processVisualBranch(const CompleteFrame& frame) {
    VisualProcessResult result;
    result.frame_id = frame.frame_id;

    thread_local std::vector<uint8_t> img_bytes_buffer_local(IMG_WIDTH * IMG_HEIGHT * 3, 0);
    std::fill(img_bytes_buffer_local.begin(), img_bytes_buffer_local.end(), 0);
    size_t offset = 0;
    for (int i = 0; i < IMG_PKT_NUM; i++) {
        const auto& pkt = frame.packets[i];
        if (!pkt.empty()) {
            std::memcpy(&img_bytes_buffer_local[offset], pkt.data(), std::min(pkt.size(), (size_t)DATA_SIZE));
        }
        offset += DATA_SIZE;
    }
    cv::Mat img_rgb(IMG_HEIGHT, IMG_WIDTH, CV_8UC3, img_bytes_buffer_local.data());

    auto tagc_start = std::chrono::high_resolution_clock::now();
    cv::Mat enhanced_rgb;
    switch (enhance_mode) {
        case EnhanceMode::Tagc:
            if (ENABLE_TAGC && TAGC_FAST_SCALE > 0.0f && TAGC_FAST_SCALE < 0.99f) {
                const int fast_w = std::max(1, static_cast<int>(IMG_WIDTH * TAGC_FAST_SCALE));
                const int fast_h = std::max(1, static_cast<int>(IMG_HEIGHT * TAGC_FAST_SCALE));
                cv::Mat resized_rgb;
                cv::resize(img_rgb, resized_rgb, cv::Size(fast_w, fast_h), 0, 0, cv::INTER_LINEAR);
                cv::Mat enhanced_small = tagc.process(resized_rgb, nullptr);
                cv::resize(enhanced_small, enhanced_rgb, cv::Size(IMG_WIDTH, IMG_HEIGHT), 0, 0, cv::INTER_LINEAR);
            } else if (ENABLE_TAGC) {
                enhanced_rgb = tagc.process(img_rgb, nullptr);
            } else {
                enhanced_rgb = img_rgb;
            }
            break;
        case EnhanceMode::Clahe:
            enhanced_rgb = enhanceWithClahe(img_rgb);
            break;
        case EnhanceMode::Lut:
            enhanced_rgb = enhanceWithLutGamma(img_rgb);
            break;
        case EnhanceMode::None:
            enhanced_rgb = img_rgb;
            break;
    }
    auto tagc_end = std::chrono::high_resolution_clock::now();
    result.tagc_time_ms = std::chrono::duration<double, std::milli>(tagc_end - tagc_start).count();

    cv::Mat enhanced_bgr;
    cv::cvtColor(enhanced_rgb, enhanced_bgr, cv::COLOR_RGB2BGR);
    result.frame_bgr = enhanced_bgr;

    ensureRknnInitialized();

    if (ENABLE_RKNN_DETECT && rknn_ready.load(std::memory_order_acquire)) {
        auto rknn_start = std::chrono::high_resolution_clock::now();
        result.detections = rknn_detector.detectRgb(enhanced_rgb, RKNN_MIN_SCORE);
        auto rknn_end = std::chrono::high_resolution_clock::now();
        result.rknn_time_ms = std::chrono::duration<double, std::milli>(rknn_end - rknn_start).count();
    }

    result.detection_only_bgr = cv::Mat::zeros(enhanced_bgr.size(), enhanced_bgr.type());
    if (!result.detections.empty()) {
        rknn_detector.drawDetections(result.detection_only_bgr, result.detections);
    }

    result.valid = true;
    return result;
}

ChaparisFusionInput FrameProcessor::runChaparisStage(const std::vector<DetectionBox>& detections,
                                                     const std::vector<RadarCluster>& clusters) {
    bindCurrentThreadToCore(THREAD_CHAPARIS_CORE_ID, "chaparis");
    return chaparis_matcher.run(detections, clusters);
}

std::vector<DsFusionTarget> FrameProcessor::runDsStage(const ChaparisFusionInput& fusion_input,
                                                       const std::vector<DetectionBox>& detections,
                                                       const std::vector<RadarCluster>& clusters) {
    bindCurrentThreadToCore(THREAD_DSFUSION_CORE_ID, "ds_fusion");
    return ds_fusion_engine.run(fusion_input, detections, clusters);
}

bool FrameProcessor::exportFusionInput(uint16_t frame_id, const ChaparisFusionInput& fusion_input) const {
    char fusion_filename[256];
    snprintf(fusion_filename, sizeof(fusion_filename), "%s/fusion_%06d.txt", SAVE_DIR.c_str(), frame_id);
    return exportFusionInputTxt(fusion_filename, frame_id, fusion_input);
}

bool FrameProcessor::exportDsResult(uint16_t frame_id, const std::vector<DsFusionTarget>& ds_targets) const {
    char ds_filename[256];
    snprintf(ds_filename, sizeof(ds_filename), "%s/ds_%06d.txt", SAVE_DIR.c_str(), frame_id);
    return exportDsResultTxt(ds_filename, frame_id, ds_targets);
}

RadarProcessResult FrameProcessor::processRadarBranch(const CompleteFrame& frame) {
    RadarProcessResult result;
    result.frame_id = frame.frame_id;

    const std::vector<uint8_t>* radar_packet_ptr = (frame.packets.size() > IMG_PKT_NUM)
        ? &frame.packets[IMG_PKT_NUM]
        : nullptr;
    const bool has_radar_packet = radar_packet_ptr && !radar_packet_ptr->empty();
    if (!has_radar_packet) {
        return result;
    }

    auto radar_start = std::chrono::high_resolution_clock::now();
    bindCurrentThreadToCore(THREAD_C_CORE_ID, "thread C");
    result.raw_packet = *radar_packet_ptr;
    if (result.raw_packet.size() >= 16) {
        result.radar = radar_projector.processPacket(result.raw_packet, cv::Size(IMG_WIDTH, IMG_HEIGHT));
    }
    auto radar_end = std::chrono::high_resolution_clock::now();
    result.radar_time_ms = std::chrono::duration<double, std::milli>(radar_end - radar_start).count();
    result.valid = true;
    return result;
}

bool FrameProcessor::finalizeFrame(const CompleteFrame& frame,
                                   const VisualProcessResult& visual,
                                   const RadarProcessResult& radar) {
    if (saved_frames >= target_save_frames) return false;
    if (frame.frame_id < START_SAVE_FRAME) return true;

    
    auto save_start = std::chrono::high_resolution_clock::now();

    char det_filename[256];
    snprintf(det_filename, sizeof(det_filename), "%s/det_%06d.txt", SAVE_DIR.c_str(), frame.frame_id);
    bool det_ok = exportDetectionsTxt(det_filename, frame.frame_id, visual.detections);
        std::ostringstream det_msg;
        det_msg << "[F" << frame.frame_id << "] final det_saved=" << (det_ok ? 1 : 0)
            << " det=" << visual.detections.size();
        runtime_log::logLineFileOnly(det_msg.str());

         if (saved_frames > 0) {
             std::ostringstream timing_msg;
             timing_msg << std::fixed << std::setprecision(2)
                 << "[F" << frame.frame_id << "] timing tagc="
                 << visual.tagc_time_ms << "ms rknn=" << visual.rknn_time_ms << "ms";
             runtime_log::logLine(timing_msg.str());
         }

    if (SAVE_RADAR && radar.valid && !radar.raw_packet.empty()) {
        char txt_filename[256];
        snprintf(txt_filename, sizeof(txt_filename), "%s/radar_%06d.txt", SAVE_DIR.c_str(), frame.frame_id);
        bool radar_txt_ok = exportRadarFusionTxt(txt_filename, frame.frame_id,
                                                 radar.radar.raw_points, radar.radar.clusters);
        std::ostringstream radar_msg;
        radar_msg << "[F" << frame.frame_id << "] final radar_saved=" << (radar_txt_ok ? 1 : 0)
                  << " raw=" << radar.radar.raw_points.size()
                  << " cluster=" << radar.radar.clusters.size();
        runtime_log::logLineFileOnly(radar_msg.str());
    }

    {
        std::ostringstream save_msg;
        save_msg << "已保存第 " << frame.frame_id << " 帧";
        runtime_log::logLine(save_msg.str());
    }
    auto save_end = std::chrono::high_resolution_clock::now();
    double save_time = std::chrono::duration<double, std::milli>(save_end - save_start).count();

    {
        std::lock_guard<std::mutex> lock(stats_mutex);
        saved_frames++;
        stats.total_frames++;
        if (stats.total_frames > 1) {
            stats.total_assembly_time += radar.radar_time_ms;
            stats.total_tagc_time += visual.tagc_time_ms;
            stats.total_rknn_time += visual.rknn_time_ms;
            stats.total_save_time += save_time;
        }

        if (has_prev_complete_time &&
            prev_complete_time.time_since_epoch().count() > 0 &&
            frame.first_packet_time.time_since_epoch().count() > 0) {
            double receive_time = std::chrono::duration<double, std::milli>(
                frame.first_packet_time - prev_complete_time).count();
            if (receive_time >= 0.0) {
                stats.total_receive_time += receive_time;
                stats.total_receive_samples++;
            }
        }

        if (frame.complete_time.time_since_epoch().count() > 0) {
            prev_complete_time = frame.complete_time;
            has_prev_complete_time = true;
        }
    }
    return true;
}

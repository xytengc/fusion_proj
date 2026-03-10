#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <array>

constexpr int UDP_PORT = 32896;
constexpr int IMG_WIDTH = 640;
constexpr int IMG_HEIGHT = 480;
constexpr int IMG_PKT_NUM = 640;
constexpr int FRAME_SIZE = 641;
constexpr int PACKET_SIZE = 1453;
constexpr int DATA_SIZE = 1440;
constexpr int SAVE_FRAMES = 30;
constexpr bool SAVE_RADAR = true;
constexpr int START_SAVE_FRAME = 0;
const std::string SAVE_DIR = "saved_frames";

// Radar投影与聚类配置（可根据实际标定参数调整）
constexpr bool ENABLE_RADAR_OVERLAY = true;
constexpr float RADAR_SCALE_FACTOR = 0.01f;
constexpr float RADAR_DBSCAN_EPSILON = 1.0f;      // meters
constexpr int RADAR_DBSCAN_MIN_POINTS = 2;
constexpr float RADAR_MIN_RANGE_M = 1.0f;
constexpr float RADAR_MIN_SPEED_MS = 0.1f;
constexpr float RADAR_PROJECTION_U_OFFSET = 15.0f; // MATLAB脚本中的像素偏移

constexpr std::array<float, 9> CAMERA_INTRINSIC = {
	465.2f, 0.0f, 307.9f,
	0.0f, 463.0f, 270.0f,
	0.0f,   0.0f,   1.0f
};

constexpr std::array<float, 4> CAMERA_DISTORTION = {
	-0.385f, 0.002f, 0.0f, 0.0f
};

constexpr std::array<float, 16> RADAR_TO_CAMERA = {
	 0.0f, -1.0f,  0.0f,  0.35f,
	 0.0f,  0.0f, -1.0f,  0.0f,
	 1.0f,  0.0f,  0.0f,  0.0f,
	 0.0f,  0.0f,  0.0f,  1.0f
};

constexpr float RADAR_COLOR_MAX_RANGE = 50.0f;

// RKNN源码级集成配置
constexpr bool ENABLE_RKNN_DETECT = true;
const std::string RKNN_MODEL_PATH = "../rk3588_linux/model/best.rknn";
constexpr float RKNN_MIN_SCORE = 0.25f;
constexpr int RKNN_INSTANCE_COUNT = 1;

// 视觉链路加速配置（RK3588）
constexpr bool ENABLE_TAGC = true;
// <1.0 时先缩小再做TAGC，最后放大回原尺寸；0.5通常可显著降时延
constexpr float TAGC_FAST_SCALE = 0.5f;

// 图像保存格式：true=PPM(更稳定), false=JPG
constexpr bool SAVE_IMAGE_AS_PPM = true;

// 线程核绑定配置
// cpu0~cpu3 最大频率 1800000 → 小核（A55）
// cpu4~cpu7 最大频率 2256000 → 大核（A76）
// 小核：0-3
// 大核：4-7
constexpr int THREAD_B_WORKER_COUNT = 1;

constexpr int THREAD_A_CORE_ID = 0;   // 接收线程A（小核）
constexpr int THREAD_MAIN_CORE_ID = 1; // 主调度线程（队列汇聚+帧收尾）
constexpr int THREAD_VIS_CORE_ID = 2; // 可视化线程
constexpr int THREAD_B_CORE_START_ID = 4; // B线程绑定 4
constexpr int THREAD_CHAPARIS_CORE_ID = 5; // ChaparisMatcher
constexpr int THREAD_DSFUSION_CORE_ID = 6; // DsFusionEngine
constexpr int THREAD_B_CORE_ID = 6;   // 兼容保留
constexpr int THREAD_C_CORE_ID = 7;   // 雷达线程C（大核）




#endif // CONFIG_H
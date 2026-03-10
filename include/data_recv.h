#ifndef DATA_RECV_H
#define DATA_RECV_H


#include <vector>
#include <cstdint>
#include <chrono>
#include <mutex>
#include <opencv2/opencv.hpp>
#include "config.h"

struct PacketHeader {
    uint8_t pkt_type;
    uint16_t frame_id;
    uint64_t timestamp;
    uint16_t sub_id;
};

struct Packet {
    PacketHeader header;
    std::vector<uint8_t> data;
    std::chrono::steady_clock::time_point receive_time;
};

struct CompleteFrame {
    uint16_t frame_id;
    uint64_t timestamp;
    std::vector<std::vector<uint8_t>> packets;
    int received_count = 0;
    int image_received_count = 0;
    bool radar_received = false;
    std::chrono::steady_clock::time_point first_packet_time;
    std::chrono::steady_clock::time_point complete_time;
    std::vector<std::chrono::steady_clock::time_point> packet_arrival_times;
    CompleteFrame();
};

PacketHeader parsePacketHeader(const uint8_t* data);

#endif // DATA_RECV_H

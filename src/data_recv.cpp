#include "data_recv.h"
#include "config.h"

CompleteFrame::CompleteFrame() : packets(FRAME_SIZE), received_count(0), image_received_count(0), radar_received(false) {}

PacketHeader parsePacketHeader(const uint8_t* data) {
    PacketHeader header;
    header.pkt_type = data[0];
    header.frame_id = (data[1] << 8) | data[2];
    header.timestamp = ((uint64_t)data[3] << 56) | ((uint64_t)data[4] << 48) |
                       ((uint64_t)data[5] << 40) | ((uint64_t)data[6] << 32) |
                       ((uint64_t)data[7] << 24) | ((uint64_t)data[8] << 16) |
                       ((uint64_t)data[9] << 8) | data[10];
    header.sub_id = (data[11] << 8) | data[12];
    return header;
}

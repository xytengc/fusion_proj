import socket
import struct
import numpy as np
import cv2
import os
import time

# ===============================
# 配置参数
# ===============================
UDP_PORT = 32896
IMG_WIDTH = 640
IMG_HEIGHT = 480
IMG_PKT_NUM = 640
FRAME_SIZE = 641
PACKET_SIZE = 1453
DATA_SIZE = 1440

# 保存设置
SAVE_FRAMES = 5  # 只保存前5帧
SAVE_RADAR = True  # 是否保存雷达数据
START_SAVE_FRAME = 1  # 从哪一帧开始保存

SAVE_DIR = "saved_frames"
os.makedirs(SAVE_DIR, exist_ok=True)

# ===============================
# 创建UDP Socket
# ===============================
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT))
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)  # 8MB缓冲区

print(f"UDP Receiver started on port {UDP_PORT}...")
print(f"Will save {SAVE_FRAMES} frames starting from frame {START_SAVE_FRAME}")
print("=" * 50)

# ===============================
# 帧数据缓存和统计
# ===============================
frame_cache = {}  # {frame_id: {sub_id: data}}
saved_frames = 0
total_packets = 0
last_print_time = time.time()

# ===============================
# 数据包解析函数
# ===============================
def parse_packet_header(data):
    """解析数据包头部的13字节"""
    pkt_type = data[0]
    frame_id = struct.unpack(">H", data[1:3])[0]
    timestamp = struct.unpack(">Q", data[3:11])[0]
    sub_id = struct.unpack(">H", data[11:13])[0]
    
    return pkt_type, frame_id, timestamp, sub_id

# ===============================
# 图像颜色转换函数
# ===============================
def fix_image_colors(img_rgb):
    """
    将RGB图像转换为BGR格式（OpenCV使用）
    方法1：使用OpenCV的cvtColor（推荐）
    """
    # RGB -> BGR
    img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
    return img_bgr

def fix_image_colors_manual(img_rgb):
    """
    方法2：手动翻转通道（同样有效）
    """
    # 翻转通道：R<->B, G保持不变
    img_bgr = img_rgb[:, :, ::-1]
    return img_bgr

def fix_image_colors_advanced(img_rgb):
    """
    方法3：更详细的控制
    """
    # 分离通道
    r, g, b = cv2.split(img_rgb)
    # 重新合并为BGR
    img_bgr = cv2.merge([b, g, r])
    return img_bgr

# ===============================
# 保存帧数据函数
# ===============================
def save_frame_data(frame_id, frame_data):
    """保存一帧的图像和雷达数据"""
    global saved_frames
    
    if saved_frames >= SAVE_FRAMES:
        return False
    
    if frame_id < START_SAVE_FRAME:
        return True  # 继续处理但不保存
    
    print(f"\nSaving frame {frame_id} ({saved_frames + 1}/{SAVE_FRAMES})...")
    
    try:
        # 1. 重建图像
        img_bytes = bytearray()
        for i in range(IMG_PKT_NUM):
            if i in frame_data:
                img_bytes.extend(frame_data[i])
            else:
                img_bytes.extend(bytes(DATA_SIZE))
        
        # 转换为图像
        if len(img_bytes) >= IMG_WIDTH * IMG_HEIGHT * 3:
            img_data = np.frombuffer(img_bytes[:IMG_WIDTH * IMG_HEIGHT * 3], dtype=np.uint8)
            
            # FPGA发送的是RGB格式，我们需要转换为BGR
            img_rgb = img_data.reshape((IMG_HEIGHT, IMG_WIDTH, 3))
            
            # 方法1：使用OpenCV的cvtColor转换RGB到BGR
            img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
            
            # 或者使用方法2（更简洁）：
            # img_bgr = img_rgb[:, :, ::-1]  # 直接翻转通道
            
            # 保存图像（现在颜色正确了）
            img_filename = os.path.join(SAVE_DIR, f"frame_{frame_id:06d}.jpg")
            cv2.imwrite(img_filename, img_bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])
            print(f"  ✓ Image saved: {img_filename}")
            
            # 可选：同时保存一个颜色校正前的版本用于对比
            # debug_filename = os.path.join(SAVE_DIR, f"frame_{frame_id:06d}_rgb.jpg")
            # cv2.imwrite(debug_filename, img_rgb)  # 保存RGB原图（颜色错误）
            
        else:
            print(f"  ✗ Image data incomplete")
        
        # 2. 保存雷达数据
        if SAVE_RADAR and IMG_PKT_NUM in frame_data:
            radar_data = frame_data[IMG_PKT_NUM]
            
            # 保存原始雷达数据
            radar_filename = os.path.join(SAVE_DIR, f"radar_{frame_id:06d}.bin")
            with open(radar_filename, "wb") as f:
                f.write(radar_data)
            
            # 解析并显示雷达信息
            if len(radar_data) >= 4:
                num_points = struct.unpack(">I", radar_data[:4])[0]
                print(f"  ✓ Radar data: {num_points} points")
                
                # 保存文本格式
                txt_filename = os.path.join(SAVE_DIR, f"radar_{frame_id:06d}.txt")
                with open(txt_filename, "w") as f:
                    f.write(f"Frame: {frame_id}\n")
                    f.write(f"Points: {num_points}\n")
                    f.write("=" * 40 + "\n")
                    
                    offset = 4
                    for i in range(min(num_points, 10)):
                        if offset + 16 <= len(radar_data):
                            x, y, z, intensity = struct.unpack(">ffff", radar_data[offset:offset+16])
                            f.write(f"Point {i:2d}: x={x:7.3f}, y={y:7.3f}, z={z:7.3f}, intensity={intensity:.3f}\n")
                            offset += 16
        
        saved_frames += 1
        print(f"  Frame {frame_id} saved successfully")
        
        # 如果已保存足够帧数，打印汇总信息
        if saved_frames >= SAVE_FRAMES:
            print("\n" + "=" * 50)
            print(f"✅ Saved {saved_frames} frames successfully!")
            print(f"   Images saved to: {SAVE_DIR}/frame_*.jpg")
            if SAVE_RADAR:
                print(f"   Radar data saved to: {SAVE_DIR}/radar_*.bin")
            print("=" * 50)
        
        return True
        
    except Exception as e:
        print(f"  ✗ Error saving frame {frame_id}: {e}")
        return False

# ===============================
# 主接收循环
# ===============================
try:
    print("Waiting for data... (Press Ctrl+C to stop)")
    
    while saved_frames < SAVE_FRAMES:
        # 接收UDP数据包
        try:
            data, addr = sock.recvfrom(PACKET_SIZE)
            total_packets += 1
        except socket.timeout:
            continue
        
        # 检查数据包长度
        if len(data) != PACKET_SIZE:
            continue
        
        # 解析包头
        try:
            pkt_type, frame_id, timestamp, sub_id = parse_packet_header(data[:13])
            payload = data[13:]
            
            # 验证子包ID范围
            if sub_id >= FRAME_SIZE:
                continue
            
            # 初始化该帧的缓存
            if frame_id not in frame_cache:
                frame_cache[frame_id] = {}
            
            # 存储数据
            frame_cache[frame_id][sub_id] = payload
            
            # 定期打印进度
            current_time = time.time()
            if current_time - last_print_time > 2.0:
                print(f"Received packets: {total_packets}, Cached frames: {len(frame_cache)}, Saved: {saved_frames}/{SAVE_FRAMES}")
                last_print_time = current_time
            
            # 检查帧是否完整
            if len(frame_cache[frame_id]) == FRAME_SIZE:
                print(f"\nFrame {frame_id} complete")
                
                # 处理并保存帧数据
                should_continue = save_frame_data(frame_id, frame_cache[frame_id])
                
                # 清理缓存
                del frame_cache[frame_id]
                
                # 如果不需要继续，提前退出
                if not should_continue and saved_frames >= SAVE_FRAMES:
                    break
                
                # 清理旧缓存（只保留最近5帧的缓存）
                if len(frame_cache) > 5:
                    oldest_frames = sorted(frame_cache.keys())[:-5]
                    for old_frame in oldest_frames:
                        del frame_cache[old_frame]
                        
        except Exception as e:
            print(f"Packet parse error: {e}")
            continue

except KeyboardInterrupt:
    print(f"\n\nInterrupted by user")

finally:
    sock.close()
    
    # 打印最终统计
    print("\n" + "=" * 50)
    print("Final Statistics:")
    print(f"  Total packets received: {total_packets}")
    print(f"  Frames saved: {saved_frames}/{SAVE_FRAMES}")
    print(f"  Frames still in cache: {len(frame_cache)}")
    print("=" * 50)
    
    # 如果还有缓存的完整帧但未保存，询问是否保存
    if frame_cache and saved_frames < SAVE_FRAMES:
        response = input(f"\nFound {len(frame_cache)} cached frames. Save them? (y/n): ")
        if response.lower() == 'y':
            for frame_id in list(frame_cache.keys()):
                if len(frame_cache[frame_id]) == FRAME_SIZE:
                    save_frame_data(frame_id, frame_cache[frame_id])
                    if saved_frames >= SAVE_FRAMES:
                        break
    
    print("\nReceiver stopped. Goodbye!")
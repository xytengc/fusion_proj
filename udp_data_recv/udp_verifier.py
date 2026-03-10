#!/usr/bin/env python3
"""
UDP数据接收验证程序 - CPU核心绑定版本
功能:
1. 接收FPGA发来的图像和雷达数据
2. 与cycle/_AKBK0.txt进行数据对比验证
3. 与视频进行帧对比
4. 监测UDP传输速率和丢包率
5. 输出详细日志（不保存接收数据）
"""

import socket
import struct
import numpy as np
import cv2
import os
import sys
import time
import re
import logging
import argparse
import json
from collections import defaultdict
from datetime import datetime

# ===============================
# CPU核心亲和性配置
# ===============================
try:
    import os
    os.sched_setaffinity(0, {1})  # 绑定到核心1（小核）
    print(f"[INIT] CPU绑定成功: 核心1 (LittleCore)")
except Exception as e:
    print(f"[WARN] CPU绑定失败: {e}")

# ===============================
# 日志配置
# ===============================
log_filename = f"udp_verify_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.FileHandler(log_filename),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

logger.info("="*60)
logger.info("UDP数据接收验证程序启动")
logger.info("="*60)

# ===============================
# 配置参数
# ===============================
UDP_PORT = 32896
IMG_WIDTH = 640
IMG_HEIGHT = 480
IMG_PKT_NUM = 640  # 图像子包数量
FRAME_SIZE = 641   # 总子包数(640图像+1雷达)
PACKET_SIZE = 1453 # 每个UDP包大小
DATA_SIZE = 1440   # 每个包的数据部分大小

# 雷达配置
RADAR_TXT_PATH = 'cycle/_AKBK0.txt'
VIDEO_PATH = 'cycle/video10.mp4'

logger.info(f"UDP Port: {UDP_PORT}")
logger.info(f"Image: {IMG_WIDTH}x{IMG_HEIGHT}")
logger.info(f"Image Packets: {IMG_PKT_NUM}")

# ===============================
# 全局统计数据
# ===============================
class UDPStats:
    def __init__(self):
        self.total_bytes = 0
        self.total_packets = 0
        self.start_time = time.time()
        self.last_check_time = self.start_time
        self.last_check_bytes = 0
        self.last_check_packets = 0
        self.lost_packets = 0
        self.expected_seq = defaultdict(int)  # 每个源地址的预期序列号
        
    def update(self, addr, data_len):
        self.total_bytes += data_len
        self.total_packets += 1
        
    def check_loss(self, addr, frame_id):
        """检查是否有丢包（通过frame_id检查）"""
        if addr not in self.expected_seq:
            self.expected_seq[addr] = frame_id
        else:
            if frame_id > self.expected_seq[addr]:
                loss = frame_id - self.expected_seq[addr]
                if loss > 0:
                    self.lost_packets += loss
                self.expected_seq[addr] = frame_id
                
    def get_throughput(self):
        """获取当前吞吐量 (Mbps)"""
        elapsed = time.time() - self.start_time
        if elapsed < 1:
            return 0
        mbps = (self.total_bytes * 8) / (elapsed * 1e6)
        return mbps
    
    def get_packet_rate(self):
        """获取当前包率 (pps)"""
        elapsed = time.time() - self.start_time
        if elapsed < 1:
            return 0
        pps = self.total_packets / elapsed
        return pps
    
    def get_loss_rate(self):
        """获取丢包率 (%)"""
        if self.total_packets == 0:
            return 0
        return (self.lost_packets / (self.total_packets + self.lost_packets)) * 100

stats = UDPStats()

# ===============================
# 1. 雷达数据解析函数
# ===============================
def parse_radar_txt(txt_path):
    """解析TXT，返回字典: {frame_id: [target_list]}"""
    radar_data = {}
    current_frame = -1
    parsing_AK = False
    
    logger.info(f"开始解析雷达TXT: {txt_path}")
    
    try:
        with open(txt_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()

        for line in lines:
            line = line.strip()
            
            # 识别帧号
            frame_match = re.search(r'---\s*F\s*(\d+)', line)
            if frame_match:
                current_frame = int(frame_match.group(1))
                radar_data[current_frame] = []
                parsing_AK = False
                continue
                
            # 识别AK段（使用跟踪后数据）
            if line == 'AK':
                parsing_AK = True
                continue
            if line == 'BK' or line == '#':
                parsing_AK = False
                continue
                
            # 解析目标数据
            if parsing_AK and ':' in line and 'P' in line:
                try:
                    id_part, data_part = line.split(':')
                    obj_id = int(id_part.strip())
                    
                    p_val = float(re.search(r'P\s*([-\d.]+)', data_part).group(1))
                    r_val = float(re.search(r'R\s*([-\d.]+)', data_part).group(1))
                    v_val = float(re.search(r'V\s*([-\d.]+)', data_part).group(1))
                    a_val = float(re.search(r'A\s*([-\d.]+)', data_part).group(1))
                    
                    radar_data[current_frame].append({
                        'id': obj_id,
                        'P': p_val, 'R': r_val, 'V': v_val, 'A': a_val
                    })
                except Exception as e:
                    logger.warning(f"Frame {current_frame} 解析错误: {e}")
                    
        logger.info(f"成功加载 {len(radar_data)} 帧的雷达参考数据")
        return radar_data
        
    except FileNotFoundError:
        logger.error(f"雷达TXT文件不存在: {txt_path}")
        return {}

# ===============================
# 2. 视频数据加载
# ===============================
def load_video_frames(video_path, max_frames=200):
    """加载视频帧用于对比"""
    logger.info(f"开始加载视频: {video_path}")
    
    try:
        cap = cv2.VideoCapture(video_path)
        frames = []
        frame_count = 0
        
        while frame_count < max_frames:
            ret, frame = cap.read()
            if not ret:
                break
            
            # 缩放到目标尺寸
            frame = cv2.resize(frame, (IMG_WIDTH, IMG_HEIGHT))
            frames.append(frame)
            frame_count += 1
            
        cap.release()
        logger.info(f"成功加载 {len(frames)} 帧视频数据")
        return frames
        
    except Exception as e:
        logger.error(f"视频加载失败: {e}")
        return []

# ===============================
# 3. 数据对比验证
# ===============================
def verify_radar_data(frame_id, received_radar, reference_radar):
    """对比接收的雷达数据与参考数据"""
    
    if frame_id not in reference_radar:
        return None, "参考数据缺失"
    
    ref_targets = reference_radar[frame_id]
    
    if len(ref_targets) == 0 and len(received_radar) == 0:
        return True, "一致 (无目标)"
    
    if len(received_radar) != len(ref_targets):
        return False, f"目标数不匹配: 接收{len(received_radar)}个, 参考{len(ref_targets)}个"
    
    # 对比每个目标
    errors = []
    for i, (recv, ref) in enumerate(zip(received_radar, ref_targets)):
        # 比较各个参数（允许小误差）
        r_err = abs(recv.get('R', 0) - ref['R']) 
        v_err = abs(recv.get('V', 0) - ref['V'])
        a_err = abs(recv.get('A', 0) - ref['A'])
        p_err = abs(recv.get('P', 0) - ref['P'])
        
        if r_err > 0.5 or v_err > 0.5 or a_err > 0.5 or p_err > 5:
            errors.append(f"目标{i}: R差{r_err:.2f}m, V差{v_err:.2f}m/s, A差{a_err:.2f}°, P差{p_err:.2f}dBm")
    
    if errors:
        return False, "; ".join(errors)
    else:
        return True, f"一致 ({len(ref_targets)}个目标)"

def compare_image_frames(received_img, reference_img):
    """对比接收的图像与参考视频帧"""
    if received_img is None or reference_img is None:
        return None, "图像数据缺失"
    
    if received_img.shape != reference_img.shape:
        return False, f"尺寸不匹配: 接收{received_img.shape}, 参考{reference_img.shape}"
    
    # 计算图像差异度 (MSE)
    diff = np.mean((received_img.astype(float) - reference_img.astype(float)) ** 2)
    
    if diff < 100:  # 允许一定的编码差异
        return True, f"匹配 (MSE={diff:.2f})"
    else:
        return False, f"差异较大 (MSE={diff:.2f})"

# ===============================
# 4. UDP接收与解析
# ===============================
def parse_packet_header(data):
    """解析数据包头部的13字节"""
    pkt_type = data[0]
    frame_id = struct.unpack(">H", data[1:3])[0]
    timestamp = struct.unpack(">Q", data[3:11])[0]
    sub_id = struct.unpack(">H", data[11:13])[0]
    return pkt_type, frame_id, timestamp, sub_id

def receive_and_verify(save_only=False):
    """接收数据并进行验证"""
    
    # 加载参考数据（非保存模式）
    reference_radar = {}
    reference_video = []
    if not save_only:
        reference_radar = parse_radar_txt(RADAR_TXT_PATH)
        reference_video = load_video_frames(VIDEO_PATH)
        logger.info(f"参考数据加载完成: {len(reference_radar)}帧雷达, {len(reference_video)}帧视频")
    logger.info("-" * 60)
    
    # 创建UDP Socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    
    logger.info(f"UDP接收器已启动 (端口 {UDP_PORT})")
    logger.info("等待FPGA数据...")
    logger.info("-" * 60)
    
    # 帧数据缓存
    frame_cache = defaultdict(dict)
    frame_count = 0
    last_log_time = time.time()
    last_verified_frame = -1
    
    try:
        while True:
            data, addr = sock.recvfrom(PACKET_SIZE)
            
            # 更新统计
            stats.update(addr, len(data))
            
            if len(data) < 13:
                continue
            
            try:
                pkt_type, frame_id, timestamp, sub_id = parse_packet_header(data[:13])
                payload = data[13:]
                
                stats.check_loss(addr, frame_id)
                
                if sub_id >= FRAME_SIZE:
                    continue
                
                # 验证包类型
                if sub_id < IMG_PKT_NUM:
                    if pkt_type != 0x01:
                        continue
                else:
                    if pkt_type != 0x02:
                        continue
                
                frame_cache[frame_id][sub_id] = payload
                
                # 检查帧是否完整
                if len(frame_cache[frame_id]) == FRAME_SIZE:
                    try:
                        # --- 1. 重建图像 ---
                        img_bytes = bytearray()
                        for i in range(IMG_PKT_NUM):
                            if i in frame_cache[frame_id]:
                                img_bytes.extend(frame_cache[frame_id][i])
                            else:
                                img_bytes.extend(bytes(DATA_SIZE))
                        
                        img_data = np.frombuffer(img_bytes, dtype=np.uint8)
                        
                        if len(img_data) >= IMG_WIDTH * IMG_HEIGHT * 3:
                            img_data = np.frombuffer(img_bytes[:IMG_WIDTH * IMG_HEIGHT * 3], dtype=np.uint8)
                            # FPGA发送的是RGB格式，我们需要转换为BGR
                            img_rgb = img_data.reshape((IMG_HEIGHT, IMG_WIDTH, 3))
                            # 方法1：使用OpenCV的cvtColor转换RGB到BGR
                            img = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
                        # --- 2. 处理雷达数据 ---
                        received_radar = []
                        if IMG_PKT_NUM in frame_cache[frame_id]:
                            radar_data = frame_cache[frame_id][IMG_PKT_NUM]
                            
                            if len(radar_data) >= 4:
                                try:
                                    num_points = struct.unpack("<I", radar_data[:4])[0]
                                    
                                    offset = 4
                                    for i in range(min(num_points, 64)):
                                        if offset + 16 <= len(radar_data):
                                            vals = struct.unpack_from('<HhhhHHHH', radar_data, offset)
                                            obj_id, r_int, v_int, a_int, p_int = vals[0], vals[1], vals[2], vals[3], vals[4]
                                            
                                            received_radar.append({
                                                'id': obj_id,
                                                'R': r_int / 100.0,
                                                'V': v_int / 100.0,
                                                'A': a_int / 100.0,
                                                'P': p_int / 100.0
                                            })
                                            offset += 16
                                except struct.error:
                                    pass
                        
                        # 如果是仅保存模式，则把重建的数据保存到文件并跳过验证
                        if save_only:
                            os.makedirs('frames', exist_ok=True)
                            img_path = os.path.join('frames', f"frame_{frame_id:06d}.png")
                            radar_path = os.path.join('frames', f"frame_{frame_id:06d}_radar.json")
                            try:
                                if img is not None:
                                    cv2.imwrite(img_path, img)
                                with open(radar_path, 'w', encoding='utf-8') as rf:
                                    json.dump(received_radar, rf, ensure_ascii=False, indent=2)
                                logger.info(f"[保存] Frame#{frame_id} -> {img_path}, {radar_path}")
                            except Exception as e:
                                logger.error(f"保存Frame#{frame_id}失败: {e}")
                            # 清理缓存并继续
                            frame_count += 1
                            last_verified_frame = frame_id
                            del frame_cache[frame_id]
                            continue

                        # --- 3. 验证数据 ---
                        radar_ok, radar_msg = verify_radar_data(frame_id, received_radar, reference_radar)
                        
                        image_ok = None
                        image_msg = "为进行对比"
                        if frame_id < len(reference_video) and img is not None:
                            image_ok, image_msg = compare_image_frames(img, reference_video[frame_id])
                        
                        frame_count += 1
                        last_verified_frame = frame_id
                        
                        # --- 4. 输出日志 ---
                        current_time = time.time()
                        if current_time - last_log_time >= 1.0:  # 每秒输出一次统计
                            elapsed = current_time - stats.start_time
                            throughput = stats.get_throughput()
                            pps = stats.get_packet_rate()
                            loss_rate = stats.get_loss_rate()
                            
                            logger.info(f"[统计] 帧:{frame_count:4d} | 吞吐量:{throughput:7.2f}Mbps | 包率:{pps:8.1f}pps | 丢包率:{loss_rate:6.2f}%")
                            
                            last_log_time = current_time
                        
                        # 验证结果日志
                        radar_status = "✓" if radar_ok else "✗"
                        image_status = "✓" if image_ok else "✗" if image_ok is False else "?"
                        
                        logger.info(f"[帧验证] Frame#{frame_id:6d} | 图像:{image_status} {image_msg} | 雷达:{radar_status} {radar_msg}")
                        
                        # 清理缓存
                        del frame_cache[frame_id]
                        
                    except Exception as e:
                        logger.error(f"Frame#{frame_id} 处理异常: {e}")
                        if frame_id in frame_cache:
                            del frame_cache[frame_id]
                
            except struct.error:
                continue
            
            # 定期清理旧帧
            if len(frame_cache) > 10:
                oldest_frame = min(frame_cache.keys())
                del frame_cache[oldest_frame]
    
    except KeyboardInterrupt:
        logger.info("\n" + "="*60)
        logger.info("接收器已停止 (用户中断)")
    
    except Exception as e:
        logger.error(f"致命错误: {e}")
    
    finally:
        # 输出最终统计
        elapsed = time.time() - stats.start_time
        logger.info("="*60)
        logger.info(f"[最终统计]")
        logger.info(f"  总帧数: {frame_count}")
        logger.info(f"  总包数: {stats.total_packets}")
        logger.info(f"  总字节数: {stats.total_bytes / 1024 / 1024:.2f} MB")
        logger.info(f"  运行时长: {elapsed:.1f} 秒")
        logger.info(f"  平均吞吐量: {stats.get_throughput():.2f} Mbps")
        logger.info(f"  平均包率: {stats.get_packet_rate():.1f} pps")
        logger.info(f"  总丢包数: {stats.lost_packets}")
        logger.info(f"  丢包率: {stats.get_loss_rate():.2f}%")
        logger.info(f"  日志文件: {log_filename}")
        logger.info("="*60)
        
        sock.close()

# ===============================
# 主程序入口
# ===============================
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='UDP 接收验证程序')
    parser.add_argument('--save-only', action='store_true', help='仅接收并保存frame到 frames/，不做验证')
    args = parser.parse_args()
    try:
        receive_and_verify(save_only=args.save_only)
    except KeyboardInterrupt:
        logger.info("程序被中断")
        sys.exit(0)
    except Exception as e:
        logger.error(f"程序异常: {e}")
        sys.exit(1)

#!/usr/bin/env python3
"""
UDP数据接收验证程序 - 高级版本（带实时可视化对比）
功能:
1. CPU核心绑定接收UDP数据
2. 实时与参考数据对比，显示匹配度
3. 监测UDP速率、丢包率、延迟
4. 生成对比报告和可视化结果
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
import threading
from collections import defaultdict, deque
from datetime import datetime

# ===============================
# CPU核心亲和性配置
# ===============================
try:
    import os
    # 优先绑定小核（通常是核心2-3），如果失败则绑定核心1
    try:
        os.sched_setaffinity(0, {2})
        logger_core = 2
    except:
        os.sched_setaffinity(0, {1})
        logger_core = 1
    print(f"[INIT] CPU绑定成功: 核心{logger_core}")
except Exception as e:
    print(f"[WARN] CPU绑定失败: {e}")

# ===============================
# 日志配置
# ===============================
log_filename = f"udp_verify_adv_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.FileHandler(log_filename),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

logger.info("="*70)
logger.info("UDP数据接收验证程序 - 高级版")
logger.info("="*70)

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

RADAR_TXT_PATH = 'cycle/_AKBK0.txt'
VIDEO_PATH = 'cycle/video10.mp4'

# ===============================
# 高级统计类
# ===============================
class AdvancedStats:
    def __init__(self, window_size=100):
        self.total_bytes = 0
        self.total_packets = 0
        self.start_time = time.time()
        self.lost_packets = 0
        self.expected_seq = defaultdict(int)
        
        # 滑动窗口统计（用于计算实时速率）
        self.window_size = window_size
        self.byte_window = deque(maxlen=window_size)
        self.pkt_window = deque(maxlen=window_size)
        self.timestamp_window = deque(maxlen=window_size)
        
        # 延迟统计
        self.frame_timestamps = {}
        self.latencies = deque(maxlen=100)
        
        # 验证统计
        self.radar_match_count = 0
        self.radar_error_count = 0
        self.image_match_count = 0
        self.image_error_count = 0
        self.image_skip_count = 0
        
    def update(self, pkt_size):
        self.total_bytes += pkt_size
        self.total_packets += 1
        current_time = time.time()
        self.byte_window.append(pkt_size)
        self.pkt_window.append(1)
        self.timestamp_window.append(current_time)
        
    def get_realtime_throughput(self):
        """实时吞吐量 (Mbps)"""
        if len(self.timestamp_window) < 2:
            return 0
        time_diff = self.timestamp_window[-1] - self.timestamp_window[0]
        if time_diff < 0.1:
            return 0
        bytes_sum = sum(self.byte_window)
        mbps = (bytes_sum * 8) / (time_diff * 1e6)
        return mbps
    
    def get_realtime_pps(self):
        """实时包率 (pps)"""
        if len(self.timestamp_window) < 2:
            return 0
        time_diff = self.timestamp_window[-1] - self.timestamp_window[0]
        if time_diff < 0.1:
            return 0
        pps = len(self.pkt_window) / time_diff
        return pps
    
    def record_frame_latency(self, frame_id, latency_ms):
        """记录帧延迟"""
        self.latencies.append(latency_ms)
    
    def get_avg_latency(self):
        """平均延迟 (ms)"""
        if len(self.latencies) == 0:
            return 0
        return np.mean(list(self.latencies))

stats = AdvancedStats()

# ===============================
# 数据解析函数
# ===============================
def parse_radar_txt(txt_path):
    """解析雷达TXT"""
    radar_data = {}
    current_frame = -1
    parsing_AK = False
    
    logger.info(f"[DATA] 开始解析雷达TXT: {txt_path}")
    
    try:
        with open(txt_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()

        for line in lines:
            line = line.strip()
            
            frame_match = re.search(r'---\s*F\s*(\d+)', line)
            if frame_match:
                current_frame = int(frame_match.group(1))
                radar_data[current_frame] = []
                parsing_AK = False
                continue
                
            if line == 'AK':
                parsing_AK = True
                continue
            if line == 'BK' or line == '#':
                parsing_AK = False
                continue
                
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
                except Exception:
                    pass
                    
        logger.info(f"[DATA] 成功加载 {len(radar_data)} 帧雷达参考数据")
        return radar_data
        
    except FileNotFoundError:
        logger.error(f"[DATA] 雷达TXT文件不存在: {txt_path}")
        return {}

def load_video_frames(video_path, max_frames=300):
    """加载视频帧"""
    logger.info(f"[DATA] 开始加载视频: {video_path}")
    
    try:
        cap = cv2.VideoCapture(video_path)
        frames = []
        frame_count = 0
        
        while frame_count < max_frames:
            ret, frame = cap.read()
            if not ret:
                break
            frame = cv2.resize(frame, (IMG_WIDTH, IMG_HEIGHT))
            frames.append(frame)
            frame_count += 1
            
        cap.release()
        logger.info(f"[DATA] 成功加载 {len(frames)} 帧视频数据")
        return frames
        
    except Exception as e:
        logger.error(f"[DATA] 视频加载失败: {e}")
        return []

# ===============================
# 验证函数
# ===============================
def verify_radar_data(frame_id, received_radar, reference_radar):
    """对比雷达数据"""
    
    if frame_id not in reference_radar:
        return None, "参考缺失"
    
    ref_targets = reference_radar[frame_id]
    
    if len(ref_targets) == 0 and len(received_radar) == 0:
        return True, "无目标"
    
    if len(received_radar) != len(ref_targets):
        return False, f"数量不匹配(收{len(received_radar)},参{len(ref_targets)})"
    
    # 对比参数
    max_err = 0
    for recv, ref in zip(received_radar, ref_targets):
        r_err = abs(recv.get('R', 0) - ref['R'])
        v_err = abs(recv.get('V', 0) - ref['V'])
        a_err = abs(recv.get('A', 0) - ref['A'])
        p_err = abs(recv.get('P', 0) - ref['P'])
        
        max_err = max(max_err, r_err, v_err, a_err, p_err/5)
    
    if max_err > 1:
        return False, f"参数误差过大({max_err:.2f})"
    else:
        return True, f"一致({len(ref_targets)}目标,误差{max_err:.2f})"

def compare_image_frames(received_img, reference_img):
    """对比图像"""
    if received_img is None or reference_img is None:
        return None, "image_missing"
    
    if received_img.shape != reference_img.shape:
        return False, f"尺寸不匹配"
    
    diff = np.mean((received_img.astype(float) - reference_img.astype(float)) ** 2)
    similarity = 100 * np.exp(-diff / 5000)  # 转换为相似度百分比
    
    if similarity > 85:
        return True, f"匹配({similarity:.1f}%)"
    elif similarity > 70:
        return False, f"相似度低({similarity:.1f}%)"
    else:
        return False, f"差异大({similarity:.1f}%)"

def parse_packet_header(data):
    """解析包头"""
    pkt_type = data[0]
    frame_id = struct.unpack(">H", data[1:3])[0]
    timestamp = struct.unpack(">Q", data[3:11])[0]
    sub_id = struct.unpack(">H", data[11:13])[0]
    return pkt_type, frame_id, timestamp, sub_id

# ===============================
# 主接收线程
# ===============================
def receive_and_verify():
    """接收并验证数据"""
    
    # 加载参考数据
    reference_radar = parse_radar_txt(RADAR_TXT_PATH)
    reference_video = load_video_frames(VIDEO_PATH)
    
    logger.info(f"[INIT] 参考数据已加载: {len(reference_radar)}帧雷达, {len(reference_video)}帧视频")
    logger.info("-"*70)
    
    # 创建UDP Socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    
    logger.info(f"[NET] UDP接收器启动 (端口 {UDP_PORT})")
    logger.info("[NET] 等待FPGA数据...")
    logger.info("-"*70)
    
    # 帧缓存
    frame_cache = defaultdict(dict)
    frame_count = 0
    last_stats_time = time.time()
    last_detail_time = time.time()
    
    try:
        while True:
            data, addr = sock.recvfrom(PACKET_SIZE)
            
            # 更新统计
            stats.update(len(data))
            
            if len(data) < 13:
                continue
            
            try:
                pkt_type, frame_id, timestamp, sub_id = parse_packet_header(data[:13])
                payload = data[13:]
                
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
                
                # 帧完整处理
                if len(frame_cache[frame_id]) == FRAME_SIZE:
                    try:
                        # 1. 重建图像
                        img_bytes = bytearray()
                        for i in range(IMG_PKT_NUM):
                            if i in frame_cache[frame_id]:
                                img_bytes.extend(frame_cache[frame_id][i])
                            else:
                                img_bytes.extend(bytes(DATA_SIZE))
                        
                        img_data = np.frombuffer(img_bytes, dtype=np.uint8)
                        img = None
                        
                        if len(img_data) >= IMG_WIDTH * IMG_HEIGHT * 3:
                            img_data = np.frombuffer(img_bytes[:IMG_WIDTH * IMG_HEIGHT * 3], dtype=np.uint8)
                            # FPGA发送的是RGB格式，我们需要转换为BGR
                            img_rgb = img_data.reshape((IMG_HEIGHT, IMG_WIDTH, 3))
                            # 方法1：使用OpenCV的cvtColor转换RGB到BGR
                            img = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
                        
                        # 2. 解析雷达
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
                        
                        # 3. 验证
                        radar_ok, radar_msg = verify_radar_data(frame_id, received_radar, reference_radar)
                        if radar_ok:
                            stats.radar_match_count += 1
                        elif radar_ok is False:
                            stats.radar_error_count += 1
                        
                        image_ok = None
                        image_msg = "skip"
                        if frame_id < len(reference_video) and img is not None:
                            image_ok, image_msg = compare_image_frames(img, reference_video[frame_id])
                            if image_ok:
                                stats.image_match_count += 1
                            elif image_ok is False:
                                stats.image_error_count += 1
                        else:
                            stats.image_skip_count += 1
                        
                        frame_count += 1
                        
                        # 实时统计输出
                        current_time = time.time()
                        if current_time - last_stats_time >= 2.0:
                            throughput = stats.get_realtime_throughput()
                            pps = stats.get_realtime_pps()
                            
                            logger.info(
                                f"[STATS] 帧:{frame_count:4d} | "
                                f"吞吐:{throughput:7.2f}Mbps | "
                                f"包率:{pps:7.0f}pps | "
                                f"雷达✓:{stats.radar_match_count:3d} | "
                                f"图像✓:{stats.image_match_count:3d}"
                            )
                            last_stats_time = current_time
                        
                        # 详细验证日志
                        if current_time - last_detail_time >= 5.0:
                            radar_status = "✓" if radar_ok else ("✗" if radar_ok is False else "?")
                            image_status = "✓" if image_ok else ("✗" if image_ok is False else "?")
                            
                            logger.info(
                                f"[DETAIL] Frame#{frame_id:6d} | "
                                f"图:{image_status} {image_msg:12s} | "
                                f"雷:{radar_status} {radar_msg}"
                            )
                            last_detail_time = current_time
                        
                        del frame_cache[frame_id]
                        
                    except Exception as e:
                        logger.warning(f"[ERR] Frame#{frame_id} 处理异常: {e}")
                        if frame_id in frame_cache:
                            del frame_cache[frame_id]
                
            except struct.error:
                continue
            
            # 清理旧帧
            if len(frame_cache) > 15:
                oldest = min(frame_cache.keys())
                del frame_cache[oldest]
    
    except KeyboardInterrupt:
        logger.info("\n" + "="*70)
        logger.info("[INFO] 接收器已停止 (Ctrl+C)")
    
    except Exception as e:
        logger.error(f"[FATAL] 致命错误: {e}")
    
    finally:
        # 最终报告
        elapsed = time.time() - stats.start_time
        total_frames_expected = stats.total_packets / 641  # 每帧641个包
        
        logger.info("="*70)
        logger.info("[REPORT] 性能统计")
        logger.info("-"*70)
        logger.info(f"运行时长: {elapsed:.1f}s")
        logger.info(f"接收总包数: {stats.total_packets}")
        logger.info(f"接收总字节: {stats.total_bytes / 1024 / 1024:.2f} MB")
        logger.info(f"完整帧数: {frame_count}")
        logger.info(f"平均吞吐量: {(stats.total_bytes * 8) / (elapsed * 1e6):.2f} Mbps")
        logger.info(f"平均包率: {stats.total_packets / elapsed:.0f} pps")
        logger.info("-"*70)
        logger.info("[REPORT] 验证统计")
        logger.info("-"*70)
        logger.info(f"雷达数据匹配: {stats.radar_match_count}/{frame_count}")
        logger.info(f"雷达数据错误: {stats.radar_error_count}/{frame_count}")
        logger.info(f"图像数据匹配: {stats.image_match_count}/{frame_count}")
        logger.info(f"图像数据错误: {stats.image_error_count}/{frame_count}")
        logger.info(f"图像数据跳过: {stats.image_skip_count}/{frame_count}")
        
        if frame_count > 0:
            radar_acc = (stats.radar_match_count / frame_count) * 100
            image_acc = (stats.image_match_count / (frame_count - stats.image_skip_count)) * 100 if stats.image_skip_count < frame_count else 0
            logger.info(f"雷达准确率: {radar_acc:.1f}%")
            logger.info(f"图像准确率: {image_acc:.1f}%")
        
        logger.info("="*70)
        logger.info(f"[INFO] 日志已保存: {log_filename}")
        logger.info("="*70)
        
        sock.close()

# ===============================
# 主程序入口
# ===============================
if __name__ == '__main__':
    try:
        receive_and_verify()
    except KeyboardInterrupt:
        logger.info("程序被中断")
        sys.exit(0)
    except Exception as e:
        logger.error(f"程序异常: {e}")
        sys.exit(1)

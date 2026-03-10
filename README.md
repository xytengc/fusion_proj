# fusion_proj

基于 RK3588 的雷达-视觉融合处理工程，包含：
- UDP 收包与帧组装
- 图像增强 + RKNN 目标检测
- 雷达点解析 / DBSCAN 聚类 / 图像投影
- Chaparis 匹配（雷达-视觉关联）
- DS 证据融合
- 多线程流水线与核绑定优化
- 叠加原视频背景/home/cat/AI/fusion_proj/rk3588_linux/model/video10.mp4
---

## 1. 工程目标

本工程面向实时融合场景，将相机与雷达数据在同帧内完成：
1. 检测（视觉）
2. 聚类与投影（雷达）
3. 目标匹配（Chaparis）
4. 融合决策（DS Fusion）

最终输出每帧的检测结果、雷达结果、匹配结果与融合结果，便于后处理与联调验证。

---

## 2. 目录结构（关键）

- `include/`
  - `config.h`：全局配置（线程核绑定、RKNN参数、雷达参数等）
  - `frame_processor.h`：视觉/雷达处理与导出接口
  - `chaparis_matcher.h`：Chaparis 匹配接口
  - `ds_fusion.h`：DS 融合接口
- `src/`
  - `main.cpp`：主流程，多线程调度与队列汇聚
  - `frame_processor.cpp`：视觉分支、雷达分支、导出与统计
  - `radar_projection.cpp`：雷达解析、聚类、投影、归一化
  - `chaparis_matcher.cpp`：匹配代价矩阵 + 匈牙利匹配
  - `ds_fusion.cpp`：融合逻辑（match/uR/uC 三路）
  - `rknn_detector.cpp`：RKNN 推理封装
- `saved_frames/`
  - 运行后按帧输出 `det_*.txt` / `radar_*.txt` / `fusion_*.txt` / `ds_*.txt`

---

## 3. 处理流水线

### 3.1 数据路径

1. **线程A（UDP接收）**
   - 接收 `FRAME_SIZE` 子包并组装 `CompleteFrame`
   - 投递到视觉队列与雷达队列

2. **线程B（视觉分支）**
   - 图像增强（TAGC/CLAHE/LUT）
   - RKNN 检测（单实例）

3. **线程C（雷达分支）**
   - 雷达包解析为点云
   - 点过滤 + DBSCAN 聚类
   - 聚类中心投影到图像平面
   - 计算并归一化：`SignalNorm`、`Density`

4. **Chaparis 工作线程**
   - 基于扩展框 + 代价矩阵进行匹配
   - 输出 `matches / uR / uC / qR / qV`

5. **DS 工作线程**
   - 处理 matched / radar-only / vision-only 三路目标
   - 输出 `ds_result`（`x,y,w,h,confidence,source_type`）

6. **主线程（汇聚与收尾）**
   - 统计、导出、生命周期管理

---

## 4. 线程与核绑定（当前配置）

以 `include/config.h` 为准：

- `THREAD_A_CORE_ID = 0`：UDP接收
- `THREAD_MAIN_CORE_ID = 1`：主调度
- `THREAD_B_WORKER_COUNT = 1`
- `THREAD_B_CORE_START_ID = 4`：视觉线程B
- `THREAD_CHAPARIS_CORE_ID = 5`：Chaparis
- `THREAD_DSFUSION_CORE_ID = 6`：DS
- `THREAD_C_CORE_ID = 7`：雷达线程C

> 说明：修改 `config.h` 后需要**重新编译**，运行时亲和性才会生效。

---

## 5. 与 MATLAB 口径对齐说明

### 5.1 projdbscan 对齐

- DBSCAN 聚类前，对 `xy` 做标准化处理
- 密度在聚类后计算，并做帧内归一化
- Signal 采用帧内最大值归一化，导出为 `SNorm`

### 5.2 chaparis 对齐

保留核心：
- 扩展框计算
- 成本矩阵
- 匈牙利最优匹配
- 输出 `matches/uR/uC/qR/qV`

去除非核心：
- 可视化
- MATLAB mat 保存
- 二次筛选可视化分支

### 5.3 ds_fusion 对齐

实现 `run_ds_logic` 三路：
- `matches`：融合强目标
- `uR`：雷达补检
- `uC`：视觉保留

---

## 6. 输出文件格式

默认输出目录：`saved_frames/`

- `det_XXXXXX.txt`
  - 检测框：类别、分数、bbox

- `radar_XXXXXX.txt`
  - RAW_POINTS
  - PROJECTED_CLUSTERS（含 `P`、`SNorm`、`Den`）

- `fusion_XXXXXX.txt`
  - `MATCHES`
  - `UR`、`UC`
  - `QR`、`QV`

- `ds_XXXXXX.txt`
  - `x y w h confidence source_type`

---

## 7. 构建与运行

### 7.1 构建

```bash
mkdir -p build && cd build
cmake ..
cmake --build . -j
```

> 若你已有平台专用构建目录，请使用工程现有脚本/方式。

### 7.2 运行

```bash
./build/build_rk3588_linux/fusion_proc --frames 100 --enhance lut
```

可选增强模式：`tagc | clahe | lut | none`

### 7.3 运行时可配置参数（无需改代码）

当前 `main.cpp` 已支持以下命令行参数：

- `-n, --frames <N>`：本次运行目标保存帧数（覆盖 `config.h` 的 `SAVE_FRAMES`）
- `--enhance <mode>`：视觉增强模式，`mode ∈ {tagc, clahe, lut, none}`

示例：

```bash
# 保存 20 帧，使用 LUT 增强
./build/build_rk3588_linux/fusion_proc --frames 20 --enhance lut

# 保存 200 帧，关闭增强
./build/build_rk3588_linux/fusion_proc -n 200 --enhance none
```

> 说明：资源采样周期当前为 `100ms`，在 `src/main.cpp` 内部默认值配置（`RuntimeResourceMonitor` 构造参数），尚未暴露为命令行参数。

---

## 8. `config.h` 参数说明（编译期配置）

下面参数都定义在 `include/config.h`，修改后需重新编译：

### 8.1 通信与帧结构

| 参数 | 含义 | 常见调整场景 |
|---|---|---|
| `UDP_PORT` | UDP 接收端口 | 发送端端口变更 |
| `IMG_WIDTH` | 图像宽度（像素） | 传感器分辨率调整 |
| `IMG_HEIGHT` | 图像高度（像素） | 传感器分辨率调整 |
| `IMG_PKT_NUM` | 图像数据子包数量 | 发包协议变化 |
| `FRAME_SIZE` | 单帧总包数（图像+雷达） | 发包协议变化 |
| `PACKET_SIZE` | 单个 UDP 包总字节数 | 发包协议变化 |
| `DATA_SIZE` | 单包有效载荷字节数 | 发包协议变化 |
| `SAVE_FRAMES` | 默认保存帧数（可被 `--frames` 覆盖） | 批量测试长度调整 |
| `SAVE_RADAR` | 是否保存雷达文本输出 | 减少 I/O 或调试雷达输出 |
| `START_SAVE_FRAME` | 从第几帧开始写盘 | 跳过预热帧 |
| `SAVE_DIR` | 输出目录 | 切换实验输出路径 |

### 8.2 雷达投影与聚类

| 参数 | 含义 | 常见调整场景 |
|---|---|---|
| `ENABLE_RADAR_OVERLAY` | 是否启用雷达投影叠加流程 | 仅调视觉链路时可关闭 |
| `RADAR_SCALE_FACTOR` | 雷达原始坐标缩放系数 | 雷达原始单位/标定口径变化 |
| `RADAR_DBSCAN_EPSILON` | DBSCAN 邻域半径（米） | 聚类更松/更紧 |
| `RADAR_DBSCAN_MIN_POINTS` | DBSCAN 最小点数阈值 | 抑制离散噪声 |
| `RADAR_MIN_RANGE_M` | 最小距离阈值（米） | 近距离噪声过滤 |
| `RADAR_MIN_SPEED_MS` | 最小速度阈值（m/s） | 静态杂波过滤 |
| `RADAR_PROJECTION_U_OFFSET` | 投影 U 轴偏移（像素） | 与 MATLAB/标定口径对齐 |
| `CAMERA_INTRINSIC` | 相机内参矩阵（3x3） | 相机标定更新 |
| `CAMERA_DISTORTION` | 相机畸变参数 | 相机标定更新 |
| `RADAR_TO_CAMERA` | 雷达到相机外参（4x4） | 联合标定更新 |
| `RADAR_COLOR_MAX_RANGE` | 叠加可视化的最大距离映射阈值 | 可视化颜色动态范围调整 |

### 8.3 RKNN 检测与视觉增强

| 参数 | 含义 | 常见调整场景 |
|---|---|---|
| `ENABLE_RKNN_DETECT` | 是否启用 RKNN 检测 | 仅做雷达链路调试 |
| `RKNN_MODEL_PATH` | RKNN 模型文件路径 | 切换模型版本 |
| `RKNN_MIN_SCORE` | 检测置信度阈值 | 控制误检/漏检平衡 |
| `RKNN_INSTANCE_COUNT` | RKNN 实例数配置 | NPU 并发策略实验 |
| `ENABLE_TAGC` | 是否允许 TAGC 增强路径 | 对比增强前后效果 |
| `TAGC_FAST_SCALE` | TAGC 快速缩放比例 | 速度-画质权衡 |
| `SAVE_IMAGE_AS_PPM` | 图像保存格式（PPM/JPG） | 调试稳定性或压缩体积 |

### 8.4 线程与核绑定

| 参数 | 含义 | 常见调整场景 |
|---|---|---|
| `THREAD_A_CORE_ID` | UDP 接收线程绑定核 | 收包抖动优化 |
| `THREAD_B_CORE_ID` | 兼容保留字段（当前主流程未直接使用） | 历史兼容 |
| `THREAD_B_WORKER_COUNT` | 视觉工作线程数量 | 视觉吞吐与资源占用平衡 |
| `THREAD_B_CORE_START_ID` | B 工作线程起始核号（按 worker_id 递增） | 多 B 线程核布局 |
| `THREAD_C_CORE_ID` | 雷达线程绑定核 | 雷达处理时延优化 |
| `THREAD_MAIN_CORE_ID` | 主调度线程绑定核 | 汇聚/调度稳定性优化 |
| `THREAD_CHAPARIS_CORE_ID` | Chaparis 线程绑定核 | 匹配阶段加速 |
| `THREAD_DSFUSION_CORE_ID` | DS 融合线程绑定核 | 融合阶段加速 |
| `THREAD_VIS_CORE_ID` | 可视化线程绑定核 | GUI/视频输出与主链路隔离 |

### 8.5 配置优先级建议

- **运行时优先**：`--frames`、`--enhance`
- **编译期优先**：`config.h` 其余参数（端口、标定、线程核绑定、阈值等）
- 推荐流程：先固定 `config.h`，再通过命令行改变本轮实验参数，便于复现。

---

## 9. 常见问题

### 8.1 `Failed to bind socket`

通常是端口已被旧进程占用：

```bash
ss -lunp | grep 32896
pkill -f fusion_proc
```

### 8.2 修改配置不生效

请确认已重新编译并运行新二进制。

### 8.3 线程利用率不符合预期

先检查亲和性：

```bash
PID=$(pgrep -n -f fusion_proc)
for t in /proc/$PID/task/*; do
  echo "$(basename $t) $(grep Cpus_allowed_list $t/status | awk '{print $2}')"
done
```

---

## 10. 后续建议

- 若 UDP 输入帧率较低（每帧约 1s），可进一步降低视觉并发，减少空转线程。
- 若需提升整体吞吐，优先优化 A 路接收与组帧，而不是继续增加 NPU 并发。
- 建议增加统一 profiling 脚本（线程亲和性 + 核利用率 + 各阶段耗时）用于版本回归。

---

## 11. 版本备注

本文档基于当前仓库代码状态生成，重点覆盖：
- 单实例 RKNN
- Chaparis/DS 独立工作线程
- 雷达归一化口径对齐 MATLAB

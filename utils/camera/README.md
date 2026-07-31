# Data Recorder - 多源摄像头数据采集工具

## 简介

本工具用于同步采集 **RealSense 彩色+深度** 和 **鱼眼（USB）** 图像数据，并自动保存为带时间戳的图像文件及 CSV 时间戳记录。
代码经过模块化重构，采用配置文件驱动，便于参数调整和功能扩展。

## 主要特性

- 同时采集三路数据：RealSense 彩色、RealSense 深度、鱼眼摄像头
- 支持通过 **序列号** 指定 RealSense 设备；鱼眼通过 **设备路径** 指定
- 鱼眼支持配置采集格式 `MJPG` / `YUYV`（FOURCC）
- 自动生成 episode 编号（或手动指定）
- 图像按时间戳命名；深度存为 16 位 `.png`（原始 Z16）
- 每帧时间戳及帧间延迟写入 CSV 文件
- 采集完成后输出帧率统计（平均、最小、最大、标准差）
- 多线程（生产者-消费者）设计，非阻塞保存
- RealSense / 鱼眼 **warm-up 后 barrier 同步启动**（减小启动时间偏置）
- 附带深度图伪彩色可视化脚本 `vis_depth.py`

---

## 目录结构

```
camera/
  CMakeLists.txt
  build.sh
  vis_depth.py
  config/default.yaml
  include/   (config, frame_queue, realsense_camera, fisheye_camera, recorder, saver, statistics, utils)
  src/       (main, config, realsense_camera, fisheye_camera, recorder, saver, statistics, utils)
```

---

## 依赖

- RealSense SDK 2.0 (`librealsense2`)
- OpenCV (4.x+)
- yaml-cpp (0.6+)
- CMake (3.14+)
- C++17 编译器 (GCC 7+ / Clang 6+)
- 可视化脚本额外需要：Python3 + opencv-python + numpy

Ubuntu 安装依赖：

```bash
sudo apt update
sudo apt install librealsense2-dev libopencv-dev libyaml-cpp-dev cmake build-essential
```

---

## 编译与运行

### 一键编译

```bash
chmod +x build.sh
./build.sh
```

编译产物位于 `build/data_recorder`。

### 手动编译

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### 运行

```bash
./build/data_recorder [episode_name]
```

- `episode_name` 可选，默认 `auto`（自动生成 `001`, `002` ...）
- 指定时保存至 `~/pika_ros/data/<episode_name>/`

---

## 配置文件说明（config/default.yaml）

当前配置要点：

```yaml
general:
  base_data_dir: "~/pika_ros/data"
  episode: "auto"
  duration_seconds: 10
  warmup_frames: 30
  fps_target: 90

realsense:
  serial: "230322274428"
  color:
    width: 640
    height: 480
    fps: 30
  depth:
    width: 640
    height: 480
    fps: 30

fisheye:
  device: "/dev/video60"
  width: 640
  height: 480
  fps: 30
  fourcc: "MJPG"

saver:
  color_format: "png"
  depth_format: "png"
  fisheye_format: "png"
  color_quality: 90
  depth_quality: 100
  fisheye_quality: 90
```

字段说明：

- `realsense.serial`：留空 `""` 自动选第一台；填写序列号则锁定设备（另一台示例：`230322274914`）
- `realsense.*.fps`：常见支持 `90/60/30/15/5` Hz
- `fisheye.device`：设备路径。示例：gripper 用 `/dev/video60`，sensor 用 `/dev/video50`
- `fisheye.fourcc`：`MJPG` 或 `YUYV`（YUYV 最高约 30fps，MJPG 通常可更高）
- `depth_format`：建议/必须用 `png`，以保留 16 位深度

修改配置后无需重新编译。

### 设备选择

- RealSense：执行 `rs-enumerate-devices` 查看 Serial Number，填入 `realsense.serial`
- 鱼眼：通过 `fisheye.device` 指定 `/dev/videoX`
- 多路同型号时 `/dev/videoX` 可能变化，可用稳定路径，例如：
  `/dev/v4l/by-path/pci-0000:00:14.0-usb-0:2.3:1.0-video-index0`
- 注意：部分 `video50` 等符号链接可能指向 metadata 节点，采集应使用 index0 采集节点

### RealSense（D405）常用能力摘要

- 彩色/深度常见分辨率：`640x480`、`848x480` 等
- 深度格式：`Z16`（毫米量级的 16 位深度）
- 帧率常见：`90/60/30/15/5` Hz（视分辨率而定）
- 完整能力列表：`rs-enumerate-devices`

---

## 输出说明

采集完成后，在 `base_data_dir/episode/` 下生成：

- `color/`：彩色图像
- `depth/`：深度图像（16 位灰度 PNG，原始距离值；普通看图软件会几乎全黑，属正常）
- `fisheye/`：鱼眼图像
- `csv/`：`color.csv` / `depth.csv` / `fisheye.csv`（Frame_Index, Timestamp(s), Delay(s)）

控制台会输出每路流的帧率统计。

---

## 采集线程与同步启动

### 线程分工

- **主线程**：RealSense 彩色 + 深度（同一线程、同一次 `wait_for_frames()`，硬件/SDK 帧同步）
- **鱼眼线程**：独立 USB 相机，与 RealSense **无硬件同步**
- **保存线程**：color / depth / fisheye 各一，只负责写盘

### 同步启动流程（C++17 atomic + condition_variable）

两边各自预热 `warmup_frames` 后，在 barrier 汇合，再一起开始记 Frame 0：

```
主线程(RealSense)              鱼眼线程
    |                              |
 open + start                      open + start
 warm-up N 帧                      warm-up N 帧
    |                              |
    +-------- arrive barrier ------+
              (两边都到齐才放行)
    |                              |
 丢 1 帧                           丢 1 帧
 记 Frame 0..N-1                 记 Frame 0..N-1
 (仍各自 wait_for_frames)        (仍各自 cap.read)
```

说明：

- 只能减小 **启动偏置**（实测可由约数百毫秒降到约数十毫秒量级），**不能**保证同一时刻曝光
- barrier 之后仍由各相机阻塞读控制节奏；`fps_target` 只用于计算总帧数
- 任一端启动失败会 abort，避免对端在 barrier 上死等
- 精细对齐可事后按 CSV 时间戳做最近邻/插值

---

## 深度图可视化（vis_depth.py）

保存的深度是原始 Z16，不是 Viewer 里的伪彩色。可用脚本预览：

```bash
python3 vis_depth.py /path/to/depth_xxx.png
python3 vis_depth.py /path/to/depth_xxx.png --no-show
```

伪彩色与距离对应关系：

| 颜色 | 深度 |
|------|------|
| 蓝 | 0-20 cm |
| 青绿 | 20-40 cm |
| 黄红 | 40-60 cm |
| 深红 | >60 cm |
| 黑 | 无效（深度值 0） |

说明：RealSense Viewer 中的彩色深度是实时伪彩色；磁盘 PNG 存的是真实深度数据。

---

## 注意事项

- 确保 RealSense 与鱼眼已连接，且设备节点权限允许（加入 `video`/`plugdev` 组，或临时 `chmod 666 /dev/video*`）
- 多设备时确认 `realsense.serial` 与 `fisheye.device` 指向目标相机
- 鱼眼画面偏暗时，检查是否手动曝光：

```bash
v4l2-ctl -d /dev/videoX --get-ctrl=auto_exposure,exposure_time_absolute,gain
v4l2-ctl -d /dev/videoX --set-ctrl=auto_exposure=3
```

- `auto_exposure`：本设备常见 `1=手动`，`3=自动`
- `fps_target` / 相机 `fps` 需为设备实际支持的帧率
- 鱼眼若不支持 V4L2，可将代码中的 `cv::CAP_V4L2` 改为 `cv::CAP_ANY`

---

## 模块化设计

| 模块 | 说明 |
|------|------|
| Config | 加载 YAML 配置（含 serial / fourcc 等） |
| FrameQueue | 线程安全队列，解耦生产与消费 |
| RealSenseCamera / FisheyeCamera | 摄像头采集封装 |
| Recorder | 采集流程、线程与队列管理 |
| Saver | 图像保存线程 |
| Statistics | 帧率统计 |
| vis_depth.py | 深度 PNG 伪彩色预览工具 |

---

## 一键编译脚本 build.sh

```bash
#!/bin/bash
set -e
echo "=== Data Recorder Build Script ==="
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
mkdir -p build && cd build
cmake ..
make -j$(nproc)
echo "Build successful! Executable: build/data_recorder"
echo "Run with: ./data_recorder [episode_name]"
```

### 使用方法

1. `chmod +x build.sh && ./build.sh`
2. 按需修改 `config/default.yaml`
3. `./build/data_recorder` 开始采集
4. 需要时用 `python3 vis_depth.py <depth.png>` 查看深度

---

## 许可证

本工具仅供内部使用，未开源。

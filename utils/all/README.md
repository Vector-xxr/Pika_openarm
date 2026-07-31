# Data Recorder - 多源数据采集工具（相机 + Gripper）

## 简介

本工具在 `utils/camera` 基础上扩展，同步采集 **RealSense 彩色+深度**、**鱼眼（USB）** 图像，以及 **Gripper 角度**（sensor AS5047 读角 + 夹爪跟随控制），并保存带时间戳的图像 / CSV。

## 主要特性

- 同时采集三路图像：RealSense 彩色、RealSense 深度、鱼眼摄像头
- Gripper：相机 warm-up barrier 完成后启动；三线程（采集 / 控制 / 回执）+ Saver 写 `angle.csv`
- 采集期间夹爪跟随 sensor 角度（`POSITION_CTRL_MIT` 或 `POS_VEL`）
- 支持通过 **序列号** 指定 RealSense；鱼眼通过 **设备路径**；夹爪通过串口路径
- 自动生成 episode 编号（或手动指定）
- 图像按时间戳命名；深度存为 16 位 `.png`（原始 Z16）
- 采集完成后输出各流帧率统计（含 gripper）
- RealSense / 鱼眼 **warm-up 后 barrier 同步启动**，再启夹爪子系统

---

## 目录结构

```
all/
  CMakeLists.txt
  build.sh
  vis_depth.py
  config/default.yaml
  include/
    config, frame_queue, angle_queue,
    realsense_camera, fisheye_camera,
    gripper_protocol, gripper_serial_io, gripper_controller, gripper_subsystem,
    recorder, saver, statistics, utils
  src/  (对应实现)
```

---

## 依赖

- RealSense SDK 2.0 (`librealsense2`)
- OpenCV (4.x+)
- yaml-cpp (0.6+)
- Boost.System（串口）
- nlohmann_json
- CMake (3.14+)
- C++17 编译器

Ubuntu 安装依赖：

```bash
sudo apt update
sudo apt install librealsense2-dev libopencv-dev libyaml-cpp-dev \
  libboost-system-dev nlohmann-json3-dev cmake build-essential
```

---

## 编译与运行

```bash
chmod +x build.sh
./build.sh
./build/data_recorder [episode_name]
```

编译产物：`build/data_recorder`。

---

## 配置（config/default.yaml）

```yaml
gripper:
  enabled: true
  sensor_port: "/dev/ttyUSB50"   # 读 AS5047 角度
  gripper_port: "/dev/ttyUSB60"  # 控制 + motorstatus 回执
  baud_rate: 460800
  effort_mA: 1000
  velocity: 20.0                 # VELOCITY_CTRL
  mit_mode: true                 # true→cmd22 MIT, false→cmd23 POS_VEL
  ctrl_rate: 50                  # 控制下发 Hz
  capture_rate: 50               # 角度入队/写 CSV Hz；0=不限频
```

- `gripper.enabled: false` 时跳过夹爪（仅相机）
- `ctrl_rate`：向夹爪发位置指令的频率；`capture_rate`：写 `angle.csv` / 统计用的采样频率（sensor 包仍全量更新最新角供跟随）
- 串口默认与 `utils/gripper` 工具一致；按实际 udev 调整
- 其余 `general` / `realsense` / `fisheye` / `saver` 字段与 `utils/camera` 相同

---

## 输出说明

`base_data_dir/episode/` 下：

- `color/` / `depth/` / `fisheye/`：图像
- `csv/`：`color.csv` / `depth.csv` / `fisheye.csv`
- `gripper/angle.csv`：夹爪角度（相机 barrier 成功且 gripper 启用时）

`angle.csv` 表头：

```text
Frame_Index,Timestamp(s),Delay(s),Angle(rad)
```

时间戳为 **本机收到完整 sensor JSON 包** 的系统时间（串口无设备时间戳），与相机同一套 `get_system_timestamp_seconds()`。

---

## 采集线程与同步启动

```
主线程(RealSense)              鱼眼线程
    |                              |
 warm-up N 帧                      warm-up N 帧
    +-------- arrive barrier ------+
              记下 sync t0
              启动 GripperSubsystem
                ├ Capture  (读 sensor AS5047 → AngleQueue)
                ├ Control  (ENABLE / MIT|POS_VEL 跟随)
                └ Feedback (读 gripper motorstatus)
              + Gripper Saver → gripper/angle.csv
    |                              |
 采集 Frame 0..N-1               采集 Frame 0..N-1
    |                              |
 采满 → stopGripper (DISABLE + join)
```

说明：

- 夹爪 **不做** 相机式 warm-up；仅在两侧相机预热 barrier 完成后启动
- 结束时先停夹爪再 drain 图像/角度队列
- `utils/gripper` 独立工具仍保留；本目录为抽取复用，不反向依赖其 main

---

## 深度图可视化

```bash
python3 vis_depth.py /path/to/depth_xxx.png
```

---

## 模块化设计

| 模块 | 说明 |
|------|------|
| Config | YAML（含 gripper 串口/跟随参数） |
| FrameQueue / AngleQueue | 图像帧 / 角度样本队列 |
| RealSenseCamera / FisheyeCamera | 摄像头采集 |
| GripperProtocol / SerialIO / Controller | 协议、回执、控制 |
| GripperSubsystem | 三线程门面 |
| Recorder | 流程、barrier 后启夹爪、统计 |
| Saver | 图像保存 + `save_gripper_thread_func` |

---

## 许可证

本工具仅供内部使用，未开源。

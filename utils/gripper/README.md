# Gripper Follow - sensor 夹爪跟随 gripper

## 简介

无 ROS2 的 standalone 工具：从 **sensor** 串口读取 AS5047 角度并计算距离，将 **角度** 通过 **gripper** 串口用 `POSITION_CTRL_POS_VEL`（cmd 23）下发跟随。

不需要相机式 warm-up：等到第一条完整 JSON 即可开始跟随。

---

## 数据流

1. 读 sensor JSON → `AS5047.rad`（钳位 `[0, 1.67]`）
2. 计算 `distance = 2 * (getDistance(angle) - getDistance(0))`（用于显示/观测）
3. 限频向 gripper 发送 `POSITION_CTRL_POS_VEL(angle)`

---

## 依赖

```bash
sudo apt update
sudo apt install libboost-system-dev nlohmann-json3-dev cmake build-essential
```

---

## 编译

```bash
cd utils/gripper
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

---

## 运行

```bash
# 默认：sensor=/dev/ttyUSB50  gripper=/dev/ttyUSB60
./read_gripper_angle

# 指定串口
./read_gripper_angle /dev/ttyUSB50 /dev/ttyUSB60

# 可选参数
./read_gripper_angle /dev/ttyUSB50 /dev/ttyUSB60 --effort 1000 --rate 50 --verbose
```

| 参数 | 默认 | 说明 |
|------|------|------|
| sensor_port | `/dev/ttyUSB50` | 读角度 |
| gripper_port | `/dev/ttyUSB60` | 写控制 |
| `--effort` | `1000` | 力矩上限 (mA) |
| `--rate` | `50` | 下发频率 (Hz) |
| `--verbose` | off | 打印每帧 angle/distance |

速度固定为 **0**：不发送 `VELOCITY_CTRL`（与 ROS 节点 `velocity==0` 行为一致），仅用位置指令跟随。

Ctrl+C 退出。

---

## 初始化（gripper）

1. `EFFORT_CTRL`：力矩上限（mA/1000）
2. `ENABLE`
3. 主循环：`POSITION_CTRL_POS_VEL`

波特率双方均为 `460800`。

---

## 相关代码

- 本工具：`read_gripper_angle.cpp`
- ROS 参考：`src/sensor_tools/src/serial_gripper_imu.cpp`

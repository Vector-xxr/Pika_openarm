# all_telop 运行 SOP

日常采集按下列顺序执行。设备绑定细节见 [`scripts/DEVICE_BINDING_REPORT.md`](../../scripts/DEVICE_BINDING_REPORT.md)；架构说明见 [`README.md`](README.md)。

---

## 1. 设备绑定

`all_telop` 需要：

| 逻辑口 | 用途 |
|--------|------|
| `/dev/ttyUSB50` | sensor AS5047 |
| `/dev/ttyUSB60` | gripper 电机 |
| `/dev/video60` | gripper 鱼眼 |
| RealSense SN | `config/default.yaml` 的 `realsense.serial` |

首次或换 USB 口：

```bash
conda deactivate
source ~/.bashrc
cd ~/pika_ros/scripts
python3 setup_device_myself.py   # 选 3 → 左/右手 → 只插 sensor 采样 → 只插 gripper 采样
```

口位没变、链接丢了：

```bash
bash ~/pika_ros/scripts/setup_sensor_gripper_myself.bash
```

验证：

```bash
ls -l /dev/ttyUSB50 /dev/ttyUSB60 /dev/video50 /dev/video60

stty -F /dev/ttyUSB50 460800 raw -echo
timeout 0.5 cat /dev/ttyUSB50 | head -c 200   # AS5047

stty -F /dev/ttyUSB60 460800 raw -echo
timeout 0.5 cat /dev/ttyUSB60 | head -c 200   # motor
```

---

## 2. CAN 口

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
ip link show can0   # 应为 UP
```

OpenArm CAN 夹爪默认跳过（`Runtime.skip_gripper: true`）；夹爪跟随走串口 Pika。

---

## 3. 基站校准（定位差时）

先停掉已有 locator。校准占用 tracker：

```bash
cd ~/pika_ros/install/pika_locator/lib
./survive-cli
# 强制重校：./survive-cli --force-calibrate
```

完成后退出，再开定位节点。

---

## 4. 运行

### 终端 1 — 定位

```bash
conda deactivate
source /opt/ros/humble/setup.bash
source ~/pika_ros/install/setup.bash
ros2 launch pika_locator pika_single_locator_only.launch.py
```

### 编译（改代码后）

```bash
source /opt/ros/humble/setup.bash
cd ~/pika_ros/utils/all_telop
./build.sh
```

### 终端 2 — all_telop

```bash
conda deactivate
source /opt/ros/humble/setup.bash
source ~/pika_ros/install/setup.bash
cd ~/pika_ros/utils/all_telop/build
./all_telop [episode]
# 或 ./all_telop --config ../config/default.yaml
```

### 进程内阶段

1. 相机预热 barrier 与 Vive 静态校验并行
2. 相机就绪 → Ready（静态失败仍可按 `p`，会提示 `survive-cli`）
3. Ready 后按键开录

| 键 | 作用 |
|----|------|
| `p` | 写盘 + 夹爪跟随 + 遥操 rezero |
| `q` | 暂停写盘/跟随，机械臂 hold；可再 `p` 续录 |
| `Ctrl+C` | 退出落盘，disable 夹爪与电机 |

注意：`q` → 再 `p` 间隔须 **>200 ms**（帧率统计会丢掉 >0.2s 的间隙；过短会污染统计）。详见仓库根目录 [`Capture_SOP.md`](../../Capture_SOP.md)。

数据输出：`~/pika_ros/utils/all_telop/data/{task_name}/{episode}/`（含 `color/`、`depth/`、`fisheye/`、`gripper/angle.csv`、`sensor/vive.csv`）。默认 `task_name` 为 `test`，可在 `config/default.yaml` 修改。

---

## 5. 故障排查

| 现象 | 处理 |
|------|------|
| `angle.csv` 只有表头 | 确认 `ttyUSB50` 有 `AS5047.rad`，且已按 `p` |
| 鱼眼打不开 / 黑屏 | `/dev/video60` 须绑 capture（常见偶数 `video*`），勿绑 metadata |
| 夹爪不使能（Status 非 `0x40`） | 退出占串口程序，夹爪控制器断电约 5s 后重上电 |
| RealSense 未枚举 | 核对 yaml SN（默认 `230322274428`）；否则 barrier 中止，夹爪也不启动 |
| CAN / 机械臂无响应 | 检查 `can0` UP、接线；确认 locator 有 `/pika_pose` |

隔离串口夹爪：

```bash
cd ~/pika_ros/utils/gripper/build
./read_gripper_angle /dev/ttyUSB50 /dev/ttyUSB60 \
  --effort 1000 --velocity 20 --rate 100 --mit --verbose
```

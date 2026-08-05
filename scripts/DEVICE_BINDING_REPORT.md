# all_telop 设备绑定（1 sensor + 1 gripper）

`all_telop` 只认固定路径：`/dev/ttyUSB50`（AS5047）、`/dev/ttyUSB60`（夹爪电机）、`/dev/video60`（鱼眼）、RealSense 按 yaml 里的序列号。

USB 节点名会变；用 udev 按**物理口**绑符号链接。sensor/gripper 串口都是 CH340，只能靠口位区分。

---

## 1S+1G 绑定（推荐）

```bash
conda deactivate
source ~/.bashrc
cd ~/pika_ros/scripts
python3 setup_device_myself.py
```

1. 选 **3**（一个 sensor + 一个 gripper）
2. 选左手(1) / 右手(2)（只影响 Tracker 码：`pika_L_code` / `pika_R_code`）
3. **只插 sensor** → 回车采样
4. **只插 gripper**（换口）→ 回车采样
5. 自动写并执行 `setup_sensor_gripper_myself.bash`；两套插回原口后按提示检查

不要用原版 `setup_device.py` 选项 3（会覆盖生产用 `setup_sensor_gripper.bash`）。

---

## 验证

```bash
ls -l /dev/ttyUSB50 /dev/ttyUSB60 /dev/video50 /dev/video60
# 四个链接存在，50 与 60 指向不同底层设备

stty -F /dev/ttyUSB50 460800 raw -echo
timeout 0.5 cat /dev/ttyUSB50 | head -c 200   # 应含 AS5047

stty -F /dev/ttyUSB60 460800 raw -echo
timeout 0.5 cat /dev/ttyUSB60 | head -c 200   # 应含 motor
```

`all_telop` 的 RealSense SN 以 `utils/all_telop/config/default.yaml` 为准。

---

## 日常

```bash
# 口位没变、链接丢了
bash ~/pika_ros/scripts/setup_sensor_gripper_myself.bash

# 换了 USB 口 → 必须重跑向导，不要沿用旧 KERNELS
python3 ~/pika_ros/scripts/setup_device_myself.py   # 再选 3
```

---

## Tracker 码（可选）

绑定向导读不到 Tracker USB 时用环境变量：

```bash
bash ~/pika_ros/scripts/pika_localization_setup.bash LHR-46A7FB3E LHR-F00DDDE5
source ~/.bashrc
```

本机：`pika_L_code=LHR-46A7FB3E`，`pika_R_code=LHR-F00DDDE5`。

---

## 双机 2S+2G（简述）

```bash
conda deactivate
cd ~/pika_ros/scripts
python3 setup_device.py   # 先选 1（双 sensor），再选 2（双 gripper）
```

得到 `ttyUSB50/51`、`video50/51` 与 `ttyUSB60/61`、`video60/61`。  
`all_telop` 默认只认一套 `50/60`；配完双机后不要再跑 myself 选项 3（会改回单套规则）。

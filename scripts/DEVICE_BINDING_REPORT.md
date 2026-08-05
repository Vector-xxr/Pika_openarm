# all_telop 设备绑定说明

面向：一台 **sensor** + 一台 **gripper**（`utils/all_telop`）。

---

## 1. 绑定原理

### 1.1 为什么要绑

Linux 每次插拔后，真实节点名会变（`ttyUSB0`/`ttyUSB1`、`video0`/`video8` 等）。  
`all_telop` 只认固定路径：

| 逻辑口 | 配置项 | 用途 |
|--------|--------|------|
| `/dev/ttyUSB50` | `gripper.sensor_port` | AS5047 角度 JSON |
| `/dev/ttyUSB60` | `gripper.gripper_port` | 夹爪电机控制 / motorstatus |
| `/dev/video60` | `fisheye.device` | gripper 侧鱼眼 |
| RealSense SN | `realsense.serial` | 深度相机（按序列号打开，不靠 video 符号链接） |

绑定的目标：用 **稳定符号链接** 把「逻辑口」映射到「当前在某个 USB 物理口上的设备」。

### 1.2 按物理口，不按设备编号

sensor / gripper 的串口芯片都是 CH340（`1a86:7522`），**VID/PID 无法区分角色**。  
可靠依据是 USB 拓扑中的接口路径 `KERNELS`，例如：

```text
1-2.4:1.0  →  sensor 串口  → /dev/ttyUSB50
1-1.4:1.0  →  gripper 串口 → /dev/ttyUSB60
1-2.3:1.0  →  sensor 鱼眼  → /dev/video50
1-1.3:1.0  →  gripper 鱼眼 → /dev/video60
```

（上表为当前机台 hub 口位；换口后需改脚本中的 `KERNELS`。）

补充约定：

- 鱼眼只绑定**偶数** `video*`（capture）；奇数多为 metadata，不要绑。
- `/dev/ttyUSB50` 与 `/dev/ttyUSB60` 必须指向**不同**底层 `ttyUSB*`。
- 串口有效数据波特率为 **460800**；未设波特率时 `cat` 可能像乱码。

### 1.3 机制：udev 规则 + 强制符号链接

1. 向 `/etc/udev/rules.d/` 写入规则：设备 `add` 且匹配对应 `KERNELS` 时，创建 `SYMLINK`（`ttyUSB50/60`、`video50/60`）。
2. `udevadm control --reload-rules` + `trigger`。
3. 对**已经在线**的设备，udev 往往不会补建链接，因此脚本再按口解析当前设备，执行 `ln -sfn` 强制指向。

实现脚本：`scripts/setup_sensor_gripper.bash`。

---

## 2. 正确绑定工作流

### 2.1 插线

1. sensor 与 gripper **不要插在同一个 USB 口**；各自簇内线保持相对固定。
2. 两套均上电，确认至少有两个真实串口节点、两套 DECXIN 鱼眼、夹爪侧 RealSense（SN 与 `default.yaml` 一致）。

### 2.2 执行绑定

```bash
bash ~/pika_ros/scripts/setup_sensor_gripper.bash
ls -l /dev/ttyUSB50 /dev/ttyUSB60 /dev/video50 /dev/video60
```

期望示例（底层编号可变，**角色/口位**不变）：

```text
/dev/ttyUSB50 -> ttyUSB*   # 该节点须在 1-2.4
/dev/ttyUSB60 -> ttyUSB*   # 该节点须在 1-1.4，且 ≠ 50
/dev/video50  -> video*    # 偶数节点，须在 1-2.3
/dev/video60  -> video*    # 偶数节点，须在 1-1.3
```

### 2.3 验证角色

```bash
stty -F /dev/ttyUSB50 460800 raw -echo
timeout 0.5 cat /dev/ttyUSB50 | head -c 200
# 应含 "AS5047"

stty -F /dev/ttyUSB60 460800 raw -echo
timeout 0.5 cat /dev/ttyUSB60 | head -c 200
# 应含 "motor"
```

### 2.4 换口后如何更新

查新物理路径：

```bash
udevadm info -q path -n /dev/ttyUSB0
udevadm info -q path -n /dev/video0
```

把路径里的 `x-y.z:1.0` 写回 `setup_sensor_gripper.bash` 中对应规则，再重新执行脚本。  
**插法变了却不改 KERNELS，链接会指到空口或错设备。**

### 2.5 日常使用

绑定完成后，插线口位保持不变即可复用规则。  
若重插后符号链接缺失，再跑一次同一脚本即可。

---

## 3. 与 `setup_device.py` 的区别

| | `setup_sensor_gripper.bash`（推荐用于 all_telop） | `setup_device.py` |
|--|--|--|
| 形态 | 非交互 bash，按已知口位一次性写入 | 交互式向导（选 1–4 套装类型） |
| 识别方式 | **预先写死 USB `KERNELS`**，运行时解析并强制 `ln` | **逐台插拔**：插入一台 → 读 RealSense SN / 鱼眼 / 串口 /（可选）Vive 序列号 → 推断该台所在口 → 生成 bash |
| 产物 | 直接写 `/etc/udev/rules.d/*` 并建链接 | 生成/覆盖 `setup_sensor_gripper.bash`（等）再执行；并生成对应 `start_*.bash` |
| 适用 | 口位已固定的 **1 sensor + 1 gripper** | 首次配机、双 sensor、双 gripper、头盔等多种组合 |
| 对 all_telop | 路径与 `default.yaml` 一致，流程短 | 选项 3 理论上也生成 sensor+gripper 规则，但依赖当时扫全 RealSense/定位标签等；缺设备会卡住 |
| 换口维护 | 改脚本里的 `KERNELS` 后重跑 | 需重新走插拔向导，让它重新采样口位 |

要点：

- **原理相同**：最终都是「USB 物理口 → udev SYMLINK → `/dev/ttyUSB5x` / `video5x|6x`」。
- **差异在获取口位的方式**：bash 假定口位已知；`setup_device.py` 用「只插一台时采样」自动发现口位并写脚本。
- **all_telop 日常**：口位稳定时直接用 `setup_sensor_gripper.bash`；仅在全新布线、不知道 `KERNELS` 时再用 `python3 setup_device.py`（选项 3），生成后再核对/固化脚本。

---

## 4. 配置对照（程序侧）

`utils/all_telop/config/default.yaml`：

```yaml
fisheye:
  device: "/dev/video60"
gripper:
  sensor_port: "/dev/ttyUSB50"
  gripper_port: "/dev/ttyUSB60"
  baud_rate: 460800
realsense:
  serial: "230322274428"
```

---

## 5. 双 sensor + 双 gripper 硬件配置

四套同时在线时，推荐用 `setup_device.py` **分两次**采口（不要手写四个 `KERNELS`）。  
该向导没有「2S+2G」单选项，应对应选项 **1** 与选项 **2**。

### 5.1 产物对应关系

在 `~/pika_ros/scripts` 目录下执行时：

| 步骤 | 命令选择 | 生成/覆盖的脚本 | 典型逻辑口 | 写入的 udev 规则文件 |
|------|----------|-----------------|------------|----------------------|
| 先配双 sensor | 选项 **1** | `setup_multi_sensor.bash`（及 `start_multi_sensor.bash`） | `ttyUSB50/51`、`video50/51` | `sensor_serial.rules`、`sensor_fisheye.rules` |
| 再配双 gripper | 选项 **2** | `setup_multi_gripper.bash`（及 `start_multi_gripper.bash`） | `ttyUSB60/61`、`video60/61` | `gripper_serial.rules`、`gripper_fisheye.rules` |

说明：

- 两次生成的是**不同 bash 文件**与**不同 udev 规则文件名**，选项 2 **不会**覆盖选项 1 的 `setup_multi_sensor.bash` / `sensor_*.rules`。
- 必须在 `scripts/` 下启动 py，否则 bash 会写到当前工作目录：

```bash
cd ~/pika_ros/scripts
python3 setup_device.py
```

- **不要**在四套齐备后改用选项 **3**（1S+1G）：它会重写 `sensor_*.rules` / `gripper_*.rules` 为单套规则，冲掉双机配置。
- 日常勿再跑会覆盖 `setup_sensor_gripper.bash` 的流程，除非明确要回到 1S+1G。

### 5.2 推荐流程

1. **只配双 sensor**  
   ```bash
   cd ~/pika_ros/scripts
   python3 setup_device.py   # 选 1
   ```  
   按提示：只插左 sensor → 采完 → 拔掉 → 只插右 sensor → 采完。  
   脚本会生成并执行 `setup_multi_sensor.bash`。

2. **再只配双 gripper**  
   ```bash
   python3 setup_device.py   # 选 2
   ```  
   同样单台插拔采完左右。生成并执行 `setup_multi_gripper.bash`。

3. **四套一起插上**，必要时：  
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   # 或按原口重插
   ```  
   检查链接存在且互不相同：

   ```text
   /dev/ttyUSB50  /dev/ttyUSB51   # sensor 左右串口
   /dev/video50   /dev/video51    # sensor 左右鱼眼
   /dev/ttyUSB60  /dev/ttyUSB61   # gripper 左右串口
   /dev/video60   /dev/video61    # gripper 左右鱼眼
   ```

   ```bash
   ls -l /dev/ttyUSB5{0,1} /dev/ttyUSB6{0,1} /dev/video5{0,1} /dev/video6{0,1}
   ls /etc/udev/rules.d/{sensor,gripper}_{serial,fisheye}.rules
   ```

4. **固化（绑定日常）**  
   - 保留生成的 `setup_multi_sensor.bash`、`setup_multi_gripper.bash`；口位不变则日常直接：  
     ```bash
     bash setup_multi_sensor.bash
     bash setup_multi_gripper.bash
     ```  
   - py 生成的模板通常**没有**「已在线设备强制 `ln -sfn`」；若 `trigger` 后链接仍缺，可按 1S+1G 脚本同理补强制链接，或重插对应 USB。  
   - 换口后再跑 `setup_device.py` 对应选项重采；勿混用过时的 KERNELS。

5. **启动设备（与绑定分开；按需手动执行）**  
   `setup_device.py` **不会**自动运行 `start_*.bash`，只生成文件。  
   - **绑定本身不需要**再跑 `start_*.bash`；符号链接齐了即绑定完成。  
   - 若使用仓库自带的 multi **ROS 启动栈**（`sensor_tools` 那套），在链接检查通过后**手动**：  
     ```bash
     cd ~/pika_ros/scripts
     bash start_multi_sensor.bash
     bash start_multi_gripper.bash
     ```  
   - 若改用 `all_telop_double` 等其它采集程序，按其 README 启动即可，**不必**跑上述 `start_multi_*.bash`。

### 5.3 与应用侧注意

绑定层出齐 `50/51` 与 `60/61` 后，需使用支持双机的启动/采集程序。  
单进程 `utils/all_telop` 默认只认一套 `50/60`，不会自动消费四套设备。

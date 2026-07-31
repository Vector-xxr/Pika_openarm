# pika_ros

### Agilex Robotics

[English](README.md) | 中文

ubuntu
ROS2

## 介绍

Pika 数据套装产品（以下简称Pika）是一款针对**具身智能**领域数据采集场景的**空间数据采集产品**，是一款面向通用操作、轻量化的便携式采执一体化解决方案， 由采集装置及模型推理执行器以及配套的定位基站和数据背包构成。支持高效、准确、快捷、轻量的采集机器人的空间操作数据。

Pika具备超高精度的**毫米级空间信息采集能力**，支持采集数据涵盖六自由度精准空间信息、深度信息、超广角可见光视觉信息以及夹持信息。满足具身智能领域的**数据采集多信息融合需求**。执行器可以基于采集器采集的数据用于模型推理的执行器终端。

如果您在使用过程中遇到任何问题，或者有任何建议和反馈，请通过以下方式联系我们：

- GitHub Issues: [https://github.com/agilexrobotics/pika_ros/issues](https://github.com/agilexrobotics/pika_ros/issues)
- 电子邮件: [support@agilex.ai](mailto:support@agilex.ai)

我们的技术团队将尽快回复您的问题，并提供必要的支持和帮助。

pika sdk：[https://github.com/agilexrobotics/pika_sdk](https://github.com/agilexrobotics/pika_sdk)

pika 遥操作：[https://github.com/agilexrobotics/PikaAnyArm](https://github.com/agilexrobotics/PikaAnyArm)

有关更多信息，您可以参考 [Pika 产品用户手册 beta（CN）](https://agilexsupport.yuque.com/staff-hso6mo/peoot3/vm7e26ho2hmuw0ng) 和 [PIKA使用QA查询](https://agilexsupport.yuque.com/staff-hso6mo/peoot3/ltl2m8a3crra12kg)。

## 支持的环境平台



### 软件环境

- 架构：x86_64
- 操作系统：Ubuntu22.04
- ROS：humble



## install 包用途一览

`install/` 为 colcon 编译产物。运行前请先加载环境：

```bash
# 若终端提示符带 (base)，请先退出 conda，避免 Python 3.13 与 ROS Humble(3.10) 冲突
conda deactivate

source /opt/ros/humble/setup.bash
source /home/vector/pika_ros/install/setup.bash

# 确认 python 为系统 3.10
which python3      # 应为 /usr/bin/python3
python3 --version  # 应为 3.10.x
```


| 包名                         | 作用                               |
| -------------------------- | -------------------------------- |
| **sensor_tools**           | Pika 传感器启动：鱼眼、夹爪串口/IMU、RealSense |
| **pika_locator**           | 基于 Vive/libsurvive 的毫米级位姿定位      |
| **libsurvive**             | HTC Vive Tracker 底层库（定位依赖）       |
| **realsense2_camera**      | Intel RealSense D400 相机驱动节点      |
| **realsense2_camera_msgs** | RealSense 相关自定义消息                |
| **realsense2_description** | RealSense URDF / 模型显示            |
| **data_tools**             | 数据采集、同步、发布、转 MCAP/HDF5/LeRobot   |
| **data_msgs**              | 数据采集相关自定义消息（无独立节点）               |
| **agx_arm_ctrl**           | AgileX 机械臂（Piper/Nero 等）CAN 驱动控制 |
| **agx_arm_description**    | 机械臂 URDF + RViz 显示               |
| **agx_arm_moveit**         | MoveIt 运动规划配置                    |
| **agx_arm_msgs**           | 机械臂自定义消息（无独立节点）                  |
| **pika_remote_agx_arm**    | Pika 遥操作机械臂（IK / teleop）         |




## 测试命令



### 1. 消息包（只验证接口）

```bash
ros2 interface list | grep -E 'agx_arm_msgs|data_msgs|realsense2_camera_msgs'
ros2 interface show agx_arm_msgs/msg/AgxArmStatus
```



### 2. RealSense（需接深度相机）

```bash
# 检测设备
rs-enumerate-devices

# 单相机
ros2 launch realsense2_camera rs_launch.py

要两台一起开，按序列号分命名空间：

# 终端1：sensor
ros2 launch realsense2_camera rs_launch.py \
  serial_no:=_230322274914 \
  camera_namespace:=sensor camera_name:=camera
# 终端2：gripper
ros2 launch realsense2_camera rs_launch.py \
  serial_no:=_230322274428 \
  camera_namespace:=gripper camera_name:=camera

或用套件一键（按实际端口）：

ros2 launch sensor_tools open_sensor_gripper.launch.py \
  sensor_serial_port:=/dev/ttyUSB50 \
  gripper_serial_port:=/dev/ttyUSB60 \
  sensor_fisheye_port:=50 \
  gripper_fisheye_port:=60 \
  gripper_depth_camera_no:=_230322274428

# 看话题
ros2 topic list | grep camera
ros2 topic hz /camera/color/image_raw

# 仅看 URDF 模型（无需硬件）
ros2 launch realsense2_description view_model.launch.py
```



### 3. sensor_tools（需接 Pika 传感器）

按实际设备修改串口与视频号（可用 `ls /dev/video*`、`ls /dev/ttyUSB*` 查看）。

```bash
# 仅鱼眼
ros2 launch sensor_tools open_fisheye.launch.py fisheye_port:=60

# 单套 sensor（鱼眼 + 夹爪/IMU，可按需带 RealSense）
ros2 launch sensor_tools open_single_sensor.launch.py \
  serial_port:=/dev/ttyUSB0 fisheye_port:=60

# 仅夹爪
ros2 launch sensor_tools open_single_gripper.launch.py \
  serial_port:=/dev/ttyUSB0 fisheye_port:=60

# 检查（另开终端，先 source 环境）
ros2 topic list
ros2 topic echo /gripper/data --once
ros2 topic hz /camera_fisheye/color/image_raw
```

也可使用脚本：`src/sensor_tools/scripts/start_single_sensor.bash` 等。

### 4. libsurvive + pika_locator（需基站 + Tracker）

```bash
# 底层：检测设备
survive-cli

# 单位姿定位 + RViz
ros2 launch pika_locator pika_single_locator.launch.py

# 双定位 / 头盔 / 三路
ros2 launch pika_locator pika_double_locator.launch.py
ros2 launch pika_locator pika_helmet_locator.launch.py
ros2 launch pika_locator three_locator.launch.py

# 检查位姿
ros2 topic echo /vive_pose --once
ros2 topic hz /vive_pose
```



### 5. data_tools（传感器 topic 已在跑时再采）

```bash
# 单 Pika 采集
ros2 launch data_tools run_data_capture.launch.py \
  type:=single_pika datasetDir:=/tmp/pika_data episodeIndex:=0

# 双 Pika / Aloha / 遥操
ros2 launch data_tools run_data_capture.launch.py type:=multi_pika datasetDir:=/tmp/pika_data episodeIndex:=0
ros2 launch data_tools run_data_capture.launch.py type:=aloha datasetDir:=/tmp/pika_data episodeIndex:=0
ros2 launch data_tools run_data_capture.launch.py type:=single_pika_teleop datasetDir:=/tmp/pika_data episodeIndex:=0
ros2 launch data_tools run_data_capture.launch.py type:=multi_pika_teleop datasetDir:=/tmp/pika_data episodeIndex:=0

# 同步 / 回放
ros2 launch data_tools run_data_sync.launch.py
ros2 launch data_tools run_data_publish.launch.py
```



### 6. agx_arm_description（无需真机，测模型）

```bash
ros2 launch agx_arm_description display.launch.py arm_type:=piper
ros2 launch agx_arm_description display.launch.py arm_type:=piper effector_type:=agx_gripper
ros2 launch agx_arm_description display.launch.py arm_type:=nero
```



### 7. agx_arm_moveit（仿真规划，可不接真机）

```bash
ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper
ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper effector_type:=agx_gripper
```



### 8. agx_arm_ctrl（需 CAN + 真机）

```bash
# 先配置 CAN（按实际脚本路径调整）
bash src/PikaAnyArm/agx_arm/agx_arm_ros/can_config.sh can0 1000000

# 纯驱动
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
  can_port:=can0 arm_type:=piper effector_type:=agx_gripper

# 带 RViz
ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py \
  can_port:=can0 arm_type:=piper effector_type:=agx_gripper

# 驱动 + MoveIt
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
  can_port:=can0 arm_type:=piper effector_type:=agx_gripper

# 检查
ros2 topic list | grep -E 'joint|feedback|control'
ros2 topic echo /feedback/joint_states --once
```



### 9. pika_remote_agx_arm（遥操作）

```bash
# 仿真测试（定位 + 夹爪 + 仿真臂）
ros2 launch pika_remote_agx_arm test_remote_sim.launch.py

# 单臂 Piper / Nero / PiperX 真机遥操
ros2 launch pika_remote_agx_arm teleop_single_piper.launch.py
ros2 launch pika_remote_agx_arm teleop_single_nero.launch.py
ros2 launch pika_remote_agx_arm teleop_single_piper_x.launch.py

# 双臂
ros2 launch pika_remote_agx_arm teleop_double_piper.launch.py
ros2 launch pika_remote_agx_arm teleop_double_nero.launch.py
```



## 建议冒烟顺序

1. **无硬件**：`agx_arm_description` / `agx_arm_moveit demo` / `realsense2_description view_model` / `ros2 interface list`
2. **有 RealSense**：`rs_launch.py`
3. **有 Pika 传感器**：`open_fisheye.launch.py` 或 `open_single_sensor.launch.py`
4. **有 Vive**：`pika_single_locator.launch.py`
5. **有机械臂 CAN**：`start_single_agx_arm.launch.py`
6. **全链路**：sensor + locator → `data_tools` 采集，或 `pika_remote_agx_arm` 遥操


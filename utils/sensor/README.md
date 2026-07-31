# utils/sensor — Pika → OpenArm 真机笛卡尔遥操作

纯 CMake 工程（非 colcon/ament 包）。可执行文件：

| 可执行文件 | 用途 |
| --- | --- |
| `pika_control` | Pika 位姿 → OpenArm 真机笛卡尔空间遥操作（120Hz IK + 1000Hz 关节插值/CAN），内置静态/动态位姿健康检测 |

---

## 架构

```
                 ┌──────────────────────────┐
 /pika_pose ───▶ │ PikaPoseRosIO (ROS thread)│──▶ PoseQueue
 (PoseStamped)   │  + PoseHealthMonitor tee  │
                 └──────────────────────────┘        │
                                                       ▼
                              ┌───────────────────────────────────┐
                              │ PikaAdminThread  @ ik_update_hz    │
                              │  (默认 120Hz)                       │
                              │  drain PoseQueue → CartesianMapper  │
                              │  → IkSolver("dls")::step()          │
                              │  → JointSegmentCache(q_end, grip)   │
                              └───────────────────┬─────────────────┘
                                                   │ (mutex 交接)
                              ┌───────────────────▼─────────────────┐
                              │ FollowerArmThread @ follower_hz      │
                              │  (默认 1000Hz)                        │
                              │  JointInterpolator("linear")::step() │
                              │  → RobotSystemState                  │
                              │  → Control::unilateral_step() → CAN  │
                              └───────────────────────────────────────┘
```

- **Admin 线程**（`ik_update_hz`，默认 120Hz）：处理 `p`/`q` 状态机（`teleop_rezero`/`teleop_stop_req` 原子量）、取队列最新一帧位姿、`CartesianMapper::map()` 得到期望 TCP 位姿、调用 `IkSolver::step()`（阻尼最小二乘，`ik/dls_ik.*`）求解一个新的关节段终点，写入 `JointSegmentCache`。IK 的关节种子取自 Follower 线程发布的最新命令关节角快照（`SharedQSeed`），仅在按下 `p` 的这一拍改用测得关节角重新播种。
- **Follower 线程**（`follower_hz`，默认 1000Hz）：看到 `JointSegmentCache` 的新 `seq` 后，调用 `JointInterpolator::set_segment(q_end, N)`（`N = round(follower_hz / ik_update_hz)`）；每拍 `step()` 得到 `q_cmd/dq_cmd`，写入 `RobotSystemState` 引用，再调用 `Control::unilateral_step()` 驱动 CAN。若未激活遥操作，则 `hold_at()` 冻结在当前命令位置，但仍持续调用 `unilateral_step()` 以维持电机力矩。
- **ROS IO**：`PikaPoseRosIO` 独立线程 + `MultiThreadedExecutor`，订阅位姿 topic，转换为 `PoseSample` 推入 `PoseQueue`，并 tee 给 `PoseHealthMonitor`。
- **位姿健康**：启动时静止检测（首帧等待 → settle → warmup → `analyze_stationary`）。不通过只打印基站校准提示，仍进入等待 `p`；首次 `p` 后开启帧间跳变动态告警（`analyze_jump`），`q` 不停，`Ctrl+C` 才停。
- **TCP 相对轨迹记录**（可选）：`TcpRelativeCsvWriter` 记录 `p_des/R_des` 相对首次锁定的 `T0` 的相对轨迹；`T0` 只锁定一次，**遥操作 stop/resume 不会重置**。

静止健康检查阈值：漂移 ≤ 5mm / 0.03rad；抖动 ≤ 0.15 m/s、≤ 1 rad/s、≤ 100 m/s²。

---

## 编译

```bash
source /opt/ros/humble/setup.bash
cd /home/vector/pika_ros/utils/sensor
rm -rf build && mkdir build && cd build
cmake ..
make -j$(nproc) pika_control
```

产物：`build/pika_control`。

依赖：ROS2 Humble（`rclcpp`、`geometry_msgs`）、`Eigen3`、`orocos_kdl`、`kdl_parser`、`urdfdom`、`yaml-cpp`、`Threads`。不依赖 MuJoCo。

---

## 运行

**终端 1**（起 locator，发布 `/pika_pose`）：

```bash
source /opt/ros/humble/setup.bash
source /home/vector/pika_ros/install/setup.bash
ros2 launch pika_locator pika_single_locator_only.launch.py
```

**终端 2**（真机遥操作，需 CAN-FD 已 `up` 与硬件已连接）：

```bash
# 每次重启/重插后若 can0 DOWN，先拉起：
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up

source /opt/ros/humble/setup.bash
cd /home/vector/pika_ros/utils/sensor/build
./pika_control --config ../config/pika_openarm.yaml
```

**按键**（stdin，需要 TTY）：

| 键 | 作用 |
| --- | --- |
| `p` | 启动/恢复遥操作并置零；**仅在停止态有效**（再按 `p` 忽略，须先 `q`） |
| `q` | 停止遥操作并保持位姿（动态监测不停）；**仅在遥操作态有效**（再按 `q` 忽略，须先 `p`） |
| `Ctrl+C` | 退出（停遥操作 + 动态监测） |

启动流程：电机回 home → 订阅位姿 → **静态健康检查** → Ready。静态不通过时打印：

`静态检测不通过，请运行cd ~/pika_ros/install/pika_locator/lib && ./survive-cli或cd ~/pika_ros/install/pika_locator/lib && ./survive-cli --force-calibrate进行基站校准`

然后仍等待 `p`。动态监测为帧间位移/速度/角速度/间隔门限，告警限频约 1Hz。

配置文件里 `Runtime.arm` 目前只实现 `"right"`；填 `"left"`/`"both"` 会在启动时直接报错退出（未实现）。

---

## 目录概览

```
utils/sensor/
  CMakeLists.txt
  README.md
  assets/urdf/v1.urdf
  config/
    pika_openarm.yaml
  include/
    health_check.h, pose_queue.h, pose_health_monitor.h, ...
    controller/, ik/, interpolator/, safety/, hardware/, openarm_port/, io/
  src/
    health_check.cpp, pose_health_monitor.cpp   # 静态/动态位姿健康（链进 openarm_sensor_lib）
    controller/, ik/, interpolator/, safety/, hardware/, openarm_port/, io/
    app/openarm_pika_control.cpp                # pika_control 入口
  third_party/openarm_can/
  build/         (pika_control)
```

CMake 目标：`openarm_can_local`、`openarm_sensor_lib`、`pika_control`。

---

## 风险与后续改进模块（本次未实现，仅记录）

| 风险/局限 | 现象 | 涉及模块 | 后续改进方向 |
| --- | --- | --- | --- |
| 旧方向窗口 ≈ `1/ik_update_hz` | Admin 按固定周期采样最新位姿，方向信息滞后约一个 IK 周期 | `ik_update_hz`、`PikaAdminThread` | 提升 `ik_update_hz`；或自适应插值步数 |
| 固定 `N` / 抖动 | Admin 到达间隔抖动会造成局部加减速 | `interpolator/*` | 按上一段实际耗时动态调整步数 |
| `τ=0` 静态误差 | follower 只发 MIT 位置/速度、`effort=0` | `controller/control.*` | 重力/摩擦前馈 |
| `kdl_parser` 依赖 | 系统 KDL 版本差异 | `CMakeLists.txt`、`dynamics.*` | vendoring 固定版本 |
| 无实时调度 | 无 `CAP_SYS_NICE` 时降级普通调度 | `periodic_timer_thread.hpp` | setcap/sudo 或抖动监控 |
| `Control` 类过重 | 多职责耦合 | `controller/control.*` | 按职责拆分 |
| 单份 YAML 过长 | 字段多仍在同一文件 | `config/pika_openarm.yaml` | 必要时拆分 |
| CAN 需手动 up | `pika_control` 不自动配置 SocketCAN | `Runtime.can_interface` | 启动时可选 `ip link` 拉起 CAN-FD |

**上述改进本次未做，后续按本 README 再改。**

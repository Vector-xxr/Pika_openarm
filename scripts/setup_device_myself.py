#!/usr/bin/env python3
"""Experiment copy of setup_device.py for 1 sensor + 1 gripper.

Use this file so failures can be fixed here without touching setup_device.py.
Option 3 writes setup_sensor_gripper_myself.bash / start_sensor_gripper_myself.bash
(does NOT overwrite the production setup_sensor_gripper.bash).

Run from scripts/ (prefer system python; if conda base is active, deactivate first):
  conda deactivate          # 手册要求；避免 base 里坏掉的 cv2/numpy
  cd ~/pika_ros/scripts && python3 setup_device_myself.py
  # choose 3
"""

import subprocess
import re
import os
import time
from pathlib import Path

DEVICE_LABELS = {
    "左": ("左", "Left"),
    "右": ("右", "Right"),
    "sensor": ("sensor", "sensor"),
    "gripper": ("gripper", "gripper"),
    "helmet": ("helmet", "helmet"),
}


def print_bilingual(zh, en):
    print(zh)
    print(en)


def input_bilingual(zh, en):
    print_bilingual(zh, en)
    return input()


def device_label(name, lang="zh"):
    zh, en = DEVICE_LABELS[name]
    return zh if lang == "zh" else en


def set_env_var_persistent(key, value, shell_rc="~/.bashrc"):
    rc_path = Path(shell_rc).expanduser()
    if not rc_path.exists():
        rc_path.touch()

    lines = rc_path.read_text().splitlines()
    export_line = f'export {key}={value}'
    updated = False

    for i, line in enumerate(lines):
        if line.startswith(f"export {key}="):
            lines[i] = export_line
            updated = True
            break

    if not updated:
        lines.append(export_line)

    rc_path.write_text("\n".join(lines) + "\n")
    print(f"Updated {rc_path}")


def run_command(command):
    """Run a command and return its output."""
    try:
        result = subprocess.run(command, shell=True, capture_output=True, text=True)
        return result.stdout.strip()
    except Exception as e:
        print_bilingual(
            f"执行命令时出错: {str(e)}",
            f"Error running command: {str(e)}",
        )
        return None


def get_device_info(localization_tag=True):
    """Get device information."""
    # 运行 rs-enumerate-devices 命令
    rs_output = run_command("rs-enumerate-devices -s")
    if not rs_output:
        print_bilingual(
            "无法获取到深度摄像头数据",
            "Unable to get depth camera data",
        )
        return None, None, None, None

    # 解析输出获取序列号
    serial_match = re.search(r'Intel RealSense D405\s+(\d+)', rs_output)
    if not serial_match:
        print_bilingual(
            "无法获取到深度摄像头数据",
            "Unable to get depth camera data",
        )
        return None, None, None, None
    serial_number = serial_match.group(1)

    # 运行 udevadm 命令
    ls_output = run_command("ls /dev | grep ttyUSB | grep -v ttyUSB50 | grep -v ttyUSB51 | grep -v ttyUSB60 | grep -v ttyUSB61 | grep -v ttyUSB70")
    count = ls_output.count("tty")
    if count > 1:
        print_bilingual(
            "请确保工控机只插入一个USB串口设备",
            "Please ensure only one USB serial device is connected to the industrial PC",
        )
        return None, None, None, None
    udev_output = run_command(f"udevadm info /dev/{ls_output} | grep DEVPATH")
    if not udev_output:
        print_bilingual(
            "无法获取到串口数据",
            "Unable to get serial port data",
        )
        return None, None, None, None

    # 解析 USB 路径
    usb_path = udev_output[:udev_output.find(ls_output)][:-1]  # 获取 1-13.2.4:1.0 这样的格式
    usb_path = usb_path[usb_path.rfind("/")+1:]
    # print("寻找鱼眼摄像头，请在出现鱼眼摄像头时按下s，非鱼眼摄像头则按下q(注意在图像窗口按下，不要在终端！！！)")
    # video_path = None
    # cv2.setLogLevel(0)
    # for i in range(50):
    #     cap = cv2.VideoCapture(i)
    #     fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    #     cap.set(cv2.CAP_PROP_FOURCC, fourcc)
    #     cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    #     cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    #     cap.set(cv2.CAP_PROP_FPS, 30)
    #     key = None
    #     if cap.isOpened():
    #         # print("port:", "/dev/video"+str(i))
    #         while True:
    #             ret, frame = cap.read()
    #             cv2.imshow("/dev/video"+str(i), frame)
    #             key = cv2.waitKey(1)
    #             if key & 0xFF == ord('q'):
    #                 break
    #             elif key & 0xFF == ord('s'):
    #                 break
    #     cv2.destroyAllWindows()
    #     if key is not None and key & 0xFF == ord('s'):
    #         video_path = 'video' + str(i)
    #         break
    # cv2.destroyAllWindows()
    # if video_path is None:
    #     print("无法获取到鱼眼摄像头数据")
    #     return None, None

    for i in range(50):
        # Prefer even video index (capture); odd is often UVC metadata.
        if i % 2 == 1:
            continue
        video_output1 = run_command(f"cat /sys/class/video4linux/video{i}/device/../idVendor 2>/dev/null")
        video_output2 = run_command(f"cat /sys/class/video4linux/video{i}/device/../idProduct 2>/dev/null")
        if video_output1 == "1bcf" and video_output2 == "2cd1":
            video_path = 'video' + str(i)
            break
    else:
        video_path = None

    if not video_path:
        print_bilingual(
            "无法获取到鱼眼摄像头数据",
            "Unable to get fisheye camera data",
        )
        return None, None, None, None

    udev_output = run_command(f"udevadm info /dev/{video_path} | grep DEVPATH")
    video_path = udev_output[:udev_output.find("video")][:-1]  # 获取 1-13.2.4:1.0 这样的格式
    video_path = video_path[video_path.rfind("/")+1:]

    localization_tag_serial = None
    if localization_tag:
        # Find LHR device serial (28de:2300). Watchman dongle alone is 28de:2101 — not enough.
        localization_tag_list = run_command("lsusb -d 28de:2300")
        if localization_tag_list:
            localization_tag_count = len([line for line in localization_tag_list.splitlines() if "28de:2300" in line])
            if localization_tag_count > 1:
                print_bilingual(
                    "请确保工控机只插入一个定位标签设备",
                    "Please ensure only one localization tag device is connected to the industrial PC",
                )
                return None, None, None, None

        localization_tag_output = run_command("lsusb -v -d 28de:2300 2>/dev/null")
        if not localization_tag_output:
            localization_tag_output = run_command("lsusb -v | grep 28de:2300 -A 20")
        localization_tag_serial_match = re.search(r'iSerial\s+\d+\s+([^\s]+)', localization_tag_output or "")
        if not localization_tag_serial_match:
            # USB Tracker not visible (only dongle / not plugged). Fall back to env or manual.
            # Prefer side-specific env when caller sets prefer_side to "L" or "R".
            prefer = os.environ.get("_PIKA_SETUP_PREFER_SIDE", "").strip().upper()
            if prefer == "R":
                fallback = (os.environ.get("pika_R_code") or os.environ.get("pika_code") or "").strip()
            elif prefer == "L":
                fallback = (os.environ.get("pika_L_code") or os.environ.get("pika_code") or "").strip()
            else:
                fallback = (
                    os.environ.get("pika_code")
                    or os.environ.get("pika_L_code")
                    or os.environ.get("pika_R_code")
                    or ""
                ).strip()
            print_bilingual(
                "未从 USB(28de:2300) 读到定位标签序列号（常见：只有 Watchman 接收器 28de:2101）。",
                "No Tracker iSerial from USB 28de:2300 (often only Watchman dongle 28de:2101 is present).",
            )
            if fallback:
                print_bilingual(
                    f"使用已有环境变量作为定位码: {fallback}",
                    f"Using existing environment variable as tracker code: {fallback}",
                )
                localization_tag_serial = fallback
            else:
                localization_tag_serial = input_bilingual(
                    "请手动输入定位标签码(如 LHR-46A7FB3E)，直接回车跳过（仍继续绑 USB）:",
                    "Enter tracker code (e.g. LHR-46A7FB3E), or Enter to skip (USB bind continues):",
                ).strip()
                if not localization_tag_serial:
                    print_bilingual(
                        "已跳过定位码写入；USB 口绑定将继续。",
                        "Skipped tracker code; USB port binding will continue.",
                    )
                    localization_tag_serial = ""
        else:
            localization_tag_serial = localization_tag_serial_match.group(1)

    return serial_number, usb_path, video_path, localization_tag_serial


def generate_setup_bash(left_info, right_info, select, helmet_with_tracker=False):
    if select == "1":
        path = "setup_multi_sensor.bash"
        usb_num1 = 50
        usb_num2 = 51
        name1 = "sensor_"
        name2 = "sensor_"
        to1 = ">"
        to2 = ">>"
        set_env_var_persistent("pika_L_code", left_info[3])
        set_env_var_persistent("pika_R_code", right_info[3])

    if select == "2":
        path = "setup_multi_gripper.bash"
        usb_num1 = 60
        usb_num2 = 61
        name1 = "gripper_"
        name2 = "gripper_"
        to1 = ">"
        to2 = ">>"
    if select == "3":
        path = "setup_sensor_gripper_myself.bash"
        usb_num1 = 50
        usb_num2 = 60
        name1 = "sensor_"
        name2 = "gripper_"
        to1 = ">"
        to2 = ">"
        if left_info[3]:
            set_env_var_persistent("pika_code", left_info[3])
        else:
            print_bilingual(
                "未写入 pika_code（定位码为空）",
                "Skip writing pika_code (empty tracker code)",
            )
    if select == "4":
        path = "setup_helmet.bash"
        usb_num1 = 70
        usb_num2 = None
        name1 = "helmet_"
        name2 = None
        to1 = ">"
        to2 = None
        if helmet_with_tracker:
            set_env_var_persistent("pika_H_code", left_info[3])
    """Generate setup.bash file."""
    # ATTR{index}=="0" = V4L2 capture node. The old video[0,2,4,...]* glob wrongly matches
    # video15 etc. (character class includes digit '1'), often binding metadata nodes.
    if usb_num2 is not None:
        content = f"""#!/bin/bash
set -euo pipefail

sudo tee /etc/udev/rules.d/{name1}serial.rules >/dev/null <<'EOF'
ACTION=="add", KERNELS=="{left_info[1]}", SUBSYSTEMS=="usb", MODE:="0777", SYMLINK+="ttyUSB{usb_num1}"
EOF
sudo tee /etc/udev/rules.d/{name2}serial.rules >/dev/null <<'EOF'
ACTION=="add", KERNELS=="{right_info[1]}", SUBSYSTEMS=="usb", MODE:="0777", SYMLINK+="ttyUSB{usb_num2}"
EOF
sudo tee /etc/udev/rules.d/{name1}fisheye.rules >/dev/null <<'EOF'
ACTION=="add", KERNEL=="video*", ATTR{{index}}=="0", KERNELS=="{left_info[2]}", SUBSYSTEMS=="usb", MODE:="0777", SYMLINK+="video{usb_num1}"
EOF
sudo tee /etc/udev/rules.d/{name2}fisheye.rules >/dev/null <<'EOF'
ACTION=="add", KERNEL=="video*", ATTR{{index}}=="0", KERNELS=="{right_info[2]}", SUBSYSTEMS=="usb", MODE:="0777", SYMLINK+="video{usb_num2}"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty --subsystem-match=video4linux --action=add || true

# Force symlinks for already-present devices (udev often skips).
resolve_capture_on_port() {{
  local want="$1"
  local sysdev name idx path
  for sysdev in /sys/class/video4linux/video*; do
    [[ -e "$sysdev" ]] || continue
    name=$(basename "$sysdev")
    case "$name" in video50|video51|video60|video61|video70) continue ;; esac
    idx=$(cat "$sysdev/index" 2>/dev/null || echo "")
    [[ "$idx" == "0" ]] || continue
    path=$(udevadm info -q path -n "/dev/$name" 2>/dev/null || true)
    if [[ "$path" == *"/${{want}}/"* ]]; then
      printf '%s\\n' "$name"
      return 0
    fi
  done
  return 1
}}

resolve_tty_on_port() {{
  local want="$1"
  local sysdev name path
  for sysdev in /sys/class/tty/ttyUSB*; do
    [[ -e "$sysdev" ]] || continue
    name=$(basename "$sysdev")
    case "$name" in ttyUSB50|ttyUSB51|ttyUSB60|ttyUSB61|ttyUSB70) continue ;; esac
    path=$(udevadm info -q path -n "/dev/$name" 2>/dev/null || true)
    if [[ "$path" == *"/${{want}}/"* ]]; then
      printf '%s\\n' "$name"
      return 0
    fi
  done
  return 1
}}

t1=$(resolve_tty_on_port "{left_info[1]}" || true)
t2=$(resolve_tty_on_port "{right_info[1]}" || true)
v1=$(resolve_capture_on_port "{left_info[2]}" || true)
v2=$(resolve_capture_on_port "{right_info[2]}" || true)
[[ -n "${{t1:-}}" ]] && sudo ln -sfn "$t1" /dev/ttyUSB{usb_num1}
[[ -n "${{t2:-}}" ]] && sudo ln -sfn "$t2" /dev/ttyUSB{usb_num2}
[[ -n "${{v1:-}}" ]] && sudo ln -sfn "$v1" /dev/video{usb_num1}
[[ -n "${{v2:-}}" ]] && sudo ln -sfn "$v2" /dev/video{usb_num2}

echo "==== resulting links ===="
ls -l /dev/ttyUSB{usb_num1} /dev/ttyUSB{usb_num2} /dev/video{usb_num1} /dev/video{usb_num2} 2>&1 || true
"""
    else:
        content = f"""#!/bin/bash
set -euo pipefail

sudo tee /etc/udev/rules.d/{name1}serial.rules >/dev/null <<'EOF'
ACTION=="add", KERNELS=="{left_info[1]}", SUBSYSTEMS=="usb", MODE:="0777", SYMLINK+="ttyUSB{usb_num1}"
EOF
sudo tee /etc/udev/rules.d/{name1}fisheye.rules >/dev/null <<'EOF'
ACTION=="add", KERNEL=="video*", ATTR{{index}}=="0", KERNELS=="{left_info[2]}", SUBSYSTEMS=="usb", MODE:="0777", SYMLINK+="video{usb_num1}"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty --subsystem-match=video4linux --action=add || true
echo "helmet rules written; replug if symlinks missing"
"""

    with open(path, "w") as f:
        f.write(content)
    os.chmod(path, 0o755)


def generate_start_bash(left_info, right_info, select, helmet_with_tracker=False):
    if select == "1":
        path = "start_multi_sensor.bash"
        usb_num1 = 50
        usb_num2 = 51
        content = f"""
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
camera_fps=30
camera_width=640
camera_height=480
l_depth_camera_no={left_info[0]}
r_depth_camera_no={right_info[0]}

l_serial_port=/dev/ttyUSB{usb_num1}
r_serial_port=/dev/ttyUSB{usb_num2}
sudo chmod a+rw /dev/ttyUSB*
l_fisheye_port={usb_num1}
r_fisheye_port={usb_num2}
sudo chmod a+rw /dev/video*

source /opt/ros/humble/setup.bash && cd $SCRIPT_DIR/../install/sensor_tools/share/sensor_tools/scripts/ && chmod 777 usb_camera.py
if [ -n "$1" ]; then
    source $SCRIPT_DIR/../install/setup.bash && ros2 launch sensor_tools open_multi_sensor.launch.py l_depth_camera_no:=_$l_depth_camera_no r_depth_camera_no:=_$r_depth_camera_no l_serial_port:=$l_serial_port r_serial_port:=$r_serial_port l_fisheye_port:=$l_fisheye_port r_fisheye_port:=$r_fisheye_port camera_fps:=$camera_fps camera_width:=$camera_width camera_height:=$camera_height camera_profile:=$camera_width,$camera_height,$camera_fps name:=$1 name_index:=$1_
else
    source $SCRIPT_DIR/../install/setup.bash && ros2 launch sensor_tools open_multi_sensor.launch.py l_depth_camera_no:=_$l_depth_camera_no r_depth_camera_no:=_$r_depth_camera_no l_serial_port:=$l_serial_port r_serial_port:=$r_serial_port l_fisheye_port:=$l_fisheye_port r_fisheye_port:=$r_fisheye_port camera_fps:=$camera_fps camera_width:=$camera_width camera_height:=$camera_height camera_profile:=$camera_width,$camera_height,$camera_fps
fi
                """
    if select == "2":
        path = "start_multi_gripper.bash"
        usb_num1 = 60
        usb_num2 = 61
        content = f"""
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
camera_fps=30
camera_width=640
camera_height=480
l_depth_camera_no={left_info[0]}
r_depth_camera_no={right_info[0]}

l_serial_port=/dev/ttyUSB{usb_num1}
r_serial_port=/dev/ttyUSB{usb_num2}
sudo chmod a+rw /dev/ttyUSB*
l_fisheye_port={usb_num1}
r_fisheye_port={usb_num2}
sudo chmod a+rw /dev/video*

source /opt/ros/humble/setup.bash && cd $SCRIPT_DIR/../install/sensor_tools/share/sensor_tools/scripts/ && chmod 777 usb_camera.py
if [ -n "$1" ]; then
    source $SCRIPT_DIR/../install/setup.bash && ros2 launch sensor_tools open_multi_gripper.launch.py l_depth_camera_no:=_$l_depth_camera_no r_depth_camera_no:=_$r_depth_camera_no l_serial_port:=$l_serial_port r_serial_port:=$r_serial_port l_fisheye_port:=$l_fisheye_port r_fisheye_port:=$r_fisheye_port camera_fps:=$camera_fps camera_width:=$camera_width camera_height:=$camera_height camera_profile:=$camera_width,$camera_height,$camera_fps name:=$1 name_index:=$1_
else
    source $SCRIPT_DIR/../install/setup.bash && ros2 launch sensor_tools open_multi_gripper.launch.py l_depth_camera_no:=_$l_depth_camera_no r_depth_camera_no:=_$r_depth_camera_no l_serial_port:=$l_serial_port r_serial_port:=$r_serial_port l_fisheye_port:=$l_fisheye_port r_fisheye_port:=$r_fisheye_port camera_fps:=$camera_fps camera_width:=$camera_width camera_height:=$camera_height camera_profile:=$camera_width,$camera_height,$camera_fps
fi
                """
    if select == "3":
        path = "start_sensor_gripper_myself.bash"
        usb_num1 = 50
        usb_num2 = 60
        content = f"""
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
camera_fps=30
camera_width=640
camera_height=480
gripper_depth_camera_no={right_info[0]}

sensor_serial_port=/dev/ttyUSB{usb_num1}
gripper_serial_port=/dev/ttyUSB{usb_num2}
sudo chmod a+rw /dev/ttyUSB*
gripper_fisheye_port={usb_num2}
sudo chmod a+rw /dev/video*

source /opt/ros/humble/setup.bash && cd $SCRIPT_DIR/../install/sensor_tools/share/sensor_tools/scripts/ && chmod 777 usb_camera.py
source $SCRIPT_DIR/../install/setup.bash && ros2 launch sensor_tools open_sensor_gripper.launch.py gripper_depth_camera_no:=_$gripper_depth_camera_no sensor_serial_port:=$sensor_serial_port gripper_serial_port:=$gripper_serial_port  gripper_fisheye_port:=$gripper_fisheye_port camera_fps:=$camera_fps camera_width:=$camera_width camera_height:=$camera_height camera_profile:=$camera_width,$camera_height,$camera_fps
                """
    if select == "4":
        path = "start_helmet.bash"
        usb_num1 = 70
        helmet_launch = "open_helmet_whit_tracker.launch.py" if helmet_with_tracker else "open_helmet.launch.py"
        content = f"""
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
camera_fps=30
camera_width=640
camera_height=480
helmet_depth_camera_no={left_info[0]}
helmet_serial_port=/dev/ttyUSB{usb_num1}
sudo chmod a+rw /dev/ttyUSB*
helmet_fisheye_port={usb_num1}
sudo chmod a+rw /dev/video*

source /opt/ros/humble/setup.bash && cd $SCRIPT_DIR/../install/sensor_tools/share/sensor_tools/scripts/ && chmod 777 usb_camera.py
source $SCRIPT_DIR/../install/setup.bash && ros2 launch sensor_tools {helmet_launch} depth_camera_no:=_$helmet_depth_camera_no serial_port:=$helmet_serial_port fisheye_port:=$helmet_fisheye_port camera_fps:=$camera_fps camera_width:=$camera_width camera_height:=$camera_height camera_profile:=$camera_width,$camera_height,$camera_fps
                """
    with open(path, "w") as f:
        f.write(content)
    os.chmod(path, 0o755)


def main():
    print_bilingual(
        "=== pika配置工具 (setup_device_myself，实验用) ===",
        "=== Pika Setup Tool (setup_device_myself, experimental) ===",
    )
    helmet_with_tracker = False
    select = None
    while True:
        select = input_bilingual(
            "请选择绑定\n"
            "1.两个pika sensor(手持夹爪)\n"
            "2.两个pika gripper(安装于机械臂上的夹爪)\n"
            "3.一个pika sensor 一个pika gripper\n"
            "4.一个pika helmet\n"
            "请输入：",
            "Please select binding\n"
            "1. Two pika sensors (handheld grippers)\n"
            "2. Two pika grippers (mounted on robot arm)\n"
            "3. One pika sensor and one pika gripper\n"
            "4. One pika helmet\n"
            "Enter:",
        )
        if select == "1":
            device1 = "左"
            device2 = "右"
            break
        if select == "2":
            device1 = "左"
            device2 = "右"
            break
        if select == "3":
            device1 = "sensor"
            device2 = "gripper"
            while True:
                side = input_bilingual(
                    "本套 sensor+gripper 是左手还是右手？\n"
                    "1.左手 (使用 pika_L_code)\n"
                    "2.右手 (使用 pika_R_code)\n"
                    "请输入：",
                    "Is this sensor+gripper kit left or right hand?\n"
                    "1. Left (use pika_L_code)\n"
                    "2. Right (use pika_R_code)\n"
                    "Enter:",
                ).strip()
                if side == "1":
                    os.environ["_PIKA_SETUP_PREFER_SIDE"] = "L"
                    print_bilingual(
                        f"将优先使用左手定位码 pika_L_code={os.environ.get('pika_L_code', '(空)')}",
                        f"Will prefer left tracker pika_L_code={os.environ.get('pika_L_code', '(empty)')}",
                    )
                    break
                if side == "2":
                    os.environ["_PIKA_SETUP_PREFER_SIDE"] = "R"
                    print_bilingual(
                        f"将优先使用右手定位码 pika_R_code={os.environ.get('pika_R_code', '(空)')}",
                        f"Will prefer right tracker pika_R_code={os.environ.get('pika_R_code', '(empty)')}",
                    )
                    break
                print_bilingual("请输入 1 或 2", "Please enter 1 or 2")
            break
        if select == "4":
            device1 = "helmet"
            device2 = None
            tracker_select = input_bilingual(
                "helmet是否带定位器(Tracker)？\n1.带定位器\n2.不带定位器\n请输入：",
                "Does the helmet include a tracker?\n"
                "1. With tracker\n"
                "2. Without tracker\n"
                "Enter:",
            ).strip()
            helmet_with_tracker = tracker_select == "1"
            break
        else:
            print_bilingual("请输入1、2、3或4", "Please enter 1, 2, 3, or 4")
            continue

    print_bilingual(
        f"请插入{device_label(device1)}设备，然后按回车键继续...",
        f"Please plug in the {device_label(device1, 'en')} device, then press Enter to continue...",
    )
    input()
    print_bilingual(
        f"正在获取{device_label(device1)}设备信息...",
        f"Getting {device_label(device1, 'en')} device information...",
    )
    while True:
        left_info = get_device_info(True if select == "1" or select == "3" or (select == "4" and helmet_with_tracker) else False)
        if not left_info[0]:
            print_bilingual(
                f"无法获取{device_label(device1)}设备信息，请检查设备连接，然后按回车键继续...",
                f"Unable to get {device_label(device1, 'en')} device information. "
                "Please check the device connection, then press Enter to continue...",
            )
            input()
        else:
            break
    print_bilingual(
        f"{device_label(device1)}设备信息: {left_info[0]} {left_info[1]} {left_info[2]} {left_info[3]}",
        f"{device_label(device1, 'en')} device info: {left_info[0]} {left_info[1]} {left_info[2]} {left_info[3]}",
    )

    right_info = None
    if device2 is not None:
        print_bilingual(
            f"请拔出{device_label(device1)}设备，插入{device_label(device2)}设备"
            "（注意不要插在同一个USB口，配置完成后USB口不能改变），然后按回车键继续...",
            f"Please unplug the {device_label(device1, 'en')} device and plug in the {device_label(device2, 'en')} device "
            "(do not use the same USB port; the USB port must not change after setup), then press Enter to continue...",
        )
        input()
        print_bilingual(
            f"正在获取{device_label(device2)}设备信息...",
            f"Getting {device_label(device2, 'en')} device information...",
        )
        while True:
            right_info = get_device_info(True if select == "1" else False)
            if not right_info[0]:
                print_bilingual(
                    f"无法获取{device_label(device2)}设备信息，请检查设备连接，然后按回车键继续...",
                    f"Unable to get {device_label(device2, 'en')} device information. "
                    "Please check the device connection, then press Enter to continue...",
                )
                input()
            else:
                break
        print_bilingual(
            f"{device_label(device2)}设备信息: {right_info[0]} {right_info[1]} {right_info[2]} {right_info[3]}",
            f"{device_label(device2, 'en')} device info: {right_info[0]} {right_info[1]} {right_info[2]} {right_info[3]}",
        )

    # 生成配置文件
    print_bilingual("正在生成配置文件...", "Generating configuration files...")
    generate_setup_bash(left_info, right_info, select, helmet_with_tracker)
    generate_start_bash(left_info, right_info, select, helmet_with_tracker)
    setup_path = "setup_multi_sensor.bash" if select=="1" else ("setup_multi_gripper.bash" if select=="2" else ("setup_sensor_gripper_myself.bash" if select == "3" else "setup_helmet.bash"))
    start_path = "start_multi_sensor.bash" if select=="1" else ("start_multi_gripper.bash" if select=="2" else ("start_sensor_gripper_myself.bash" if select == "3" else "start_helmet.bash"))
    print_bilingual("配置完成！已生成以下文件：", "Setup complete! The following files were generated:")
    print(f"1. {setup_path}")
    print(f"2. {start_path}")
    print_bilingual(f"执行{setup_path}", f"Running {setup_path}")
    run_command(f"bash {setup_path}")
    print_bilingual("执行完成。", "Done.")
    while True:
        print_bilingual(
            "请拔插设备，注意插入先前绑定的同一个USB口。然后按回车键检查是否绑定成功...",
            "Please unplug and replug the device into the same USB port used during binding. "
            "Then press Enter to verify the binding...",
        )
        input()
        print_bilingual("请等待...", "Please wait...")
        time.sleep(5)
        video_list = run_command("ls /dev | grep video")
        usb_list = run_command("ls /dev | grep ttyUSB")
        if (select == "1" or select == "3") and video_list.find("50") < 0:
            msg_zh = "找不到sensor鱼眼 (/dev/video50)" if select == "3" else "找不到sensor（左）鱼眼"
            msg_en = "Cannot find sensor fisheye (/dev/video50)" if select == "3" else "Cannot find sensor (left) fisheye camera"
            print_bilingual(msg_zh, msg_en)
            continue
        if (select == "1") and video_list.find("51") < 0:
            print_bilingual(
                "找不到sensor（右）鱼眼",
                "Cannot find sensor (right) fisheye camera",
            )
            continue
        if (select == "2" or select == "3") and video_list.find("60") < 0:
            msg_zh = "找不到gripper鱼眼 (/dev/video60)" if select == "3" else "找不到gripper（左）鱼眼"
            msg_en = "Cannot find gripper fisheye (/dev/video60)" if select == "3" else "Cannot find gripper (left) fisheye camera"
            print_bilingual(msg_zh, msg_en)
            continue
        if (select == "2") and video_list.find("61") < 0:
            print_bilingual(
                "找不到gripper（右）鱼眼",
                "Cannot find gripper (right) fisheye camera",
            )
            continue
        if (select == "1" or select == "3") and usb_list.find("50") < 0:
            msg_zh = "找不到sensor串口 (/dev/ttyUSB50)" if select == "3" else "找不到sensor（左）串口"
            msg_en = "Cannot find sensor serial (/dev/ttyUSB50)" if select == "3" else "Cannot find sensor (left) serial port"
            print_bilingual(msg_zh, msg_en)
            continue
        if (select == "1") and usb_list.find("51") < 0:
            print_bilingual(
                "找不到sensor（右）串口",
                "Cannot find sensor (right) serial port",
            )
            continue
        if (select == "2" or select == "3") and usb_list.find("60") < 0:
            msg_zh = "找不到gripper串口 (/dev/ttyUSB60)" if select == "3" else "找不到gripper（左）串口"
            msg_en = "Cannot find gripper serial (/dev/ttyUSB60)" if select == "3" else "Cannot find gripper (left) serial port"
            print_bilingual(msg_zh, msg_en)
            continue
        if (select == "2") and usb_list.find("61") < 0:
            print_bilingual(
                "找不到gripper（右）串口",
                "Cannot find gripper (right) serial port",
            )
            continue
        if (select == "4") and usb_list.find("70") < 0:
            print_bilingual(
                "找不到helmet串口",
                "Cannot find helmet serial port",
            )
            continue
        break
    print_bilingual("绑定成功，启动设备方法：", "Binding successful. To start the device:")
    print_bilingual(f"2. 然后运行: bash {start_path}", f"2. Then run: bash {start_path}")


if __name__ == "__main__":
    main()

#!/bin/bash
# Bind persistent symlinks for ONE sensor + ONE gripper kit (all_telop).
#
# Current hub topology (update KERNELS if you replug to another port):
#   sensor  fisheye : USB 1-2.3:1.0  -> /dev/video50
#   sensor  AS5047  : USB 1-2.4:1.0  -> /dev/ttyUSB50   (JSON with AS5047.rad @ 460800)
#   gripper fisheye : USB 1-1.3:1.0  -> /dev/video60
#   gripper motor   : USB 1-1.4:1.0  -> /dev/ttyUSB60   (motorstatus JSON @ 460800)
#
# Even-numbered video* nodes only (capture index0); skip metadata index1.

set -euo pipefail

VIDEO_EVEN='video[0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48]*'

sudo tee /etc/udev/rules.d/sensor_serial.rules >/dev/null <<EOF
ACTION=="add", KERNELS=="1-2.4:1.0", SUBSYSTEMS=="usb", MODE:="0777", SYMLINK+="ttyUSB50"
EOF

sudo tee /etc/udev/rules.d/gripper_serial.rules >/dev/null <<EOF
ACTION=="add", KERNELS=="1-1.4:1.0", SUBSYSTEMS=="usb", MODE:="0777", SYMLINK+="ttyUSB60"
EOF

sudo tee /etc/udev/rules.d/sensor_fisheye.rules >/dev/null <<EOF
ACTION=="add", KERNEL=="${VIDEO_EVEN}", KERNELS=="1-2.3:1.0", SUBSYSTEMS=="usb", MODE:="0777", SYMLINK+="video50"
EOF

sudo tee /etc/udev/rules.d/gripper_fisheye.rules >/dev/null <<EOF
ACTION=="add", KERNEL=="${VIDEO_EVEN}", KERNELS=="1-1.3:1.0", SUBSYSTEMS=="usb", MODE:="0777", SYMLINK+="video60"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty --subsystem-match=video4linux --action=add || true

# Device basename sitting on a USB interface path, e.g. 1-2.4:1.0
# Only consider real char devices (skip our own symlinks 50/60).
resolve_on_port() {
  local subsystem="$1"   # tty | video4linux
  local want="$2"        # e.g. 1-2.4:1.0
  local even_only="${3:-0}"
  local sysdev name num path
  for sysdev in /sys/class/"$subsystem"/*; do
    [[ -e "$sysdev" ]] || continue
    name=$(basename "$sysdev")
    # Skip our managed symlink names and odd metadata video nodes when requested.
    case "$name" in
      ttyUSB50|ttyUSB60|video50|video60) continue ;;
    esac
    # Only scan USB serial nodes under tty (skip ttyS*, pts, etc.)
    if [[ "$subsystem" == "tty" && "$name" != ttyUSB* ]]; then
      continue
    fi
    if [[ ! -e "/dev/$name" ]]; then
      continue
    fi
    if [[ "$even_only" == "1" ]]; then
      num=${name#video}
      [[ "$num" =~ ^[0-9]+$ ]] || continue
      (( num % 2 == 0 )) || continue
    fi
    path=$(udevadm info -q path -n "/dev/$name" 2>/dev/null || true)
    if [[ "$path" == *"/usb"*"/${want}/"* ]] || [[ "$path" == *"/${want}/"* ]]; then
      printf '%s\n' "$name"
      return 0
    fi
  done
  return 1
}

echo "Resolving devices by USB port..."
tty50=$(resolve_on_port tty "1-2.4:1.0" || true)
tty60=$(resolve_on_port tty "1-1.4:1.0" || true)
vid50=$(resolve_on_port video4linux "1-2.3:1.0" 1 || true)
vid60=$(resolve_on_port video4linux "1-1.3:1.0" 1 || true)

echo "  1-2.4 -> ${tty50:-NOT FOUND}  (want ttyUSB50)"
echo "  1-1.4 -> ${tty60:-NOT FOUND}  (want ttyUSB60)"
echo "  1-2.3 -> ${vid50:-NOT FOUND}  (want video50)"
echo "  1-1.3 -> ${vid60:-NOT FOUND}  (want video60)"

# Always force (re)create links — udev SYMLINK often does not refresh for already-present devices.
if [[ -n "${tty50:-}" ]]; then
  sudo ln -sfn "$tty50" /dev/ttyUSB50
else
  echo "ERROR: no serial on 1-2.4 (sensor AS5047)" >&2
fi
if [[ -n "${tty60:-}" ]]; then
  sudo ln -sfn "$tty60" /dev/ttyUSB60
else
  echo "ERROR: no serial on 1-1.4 (gripper motor)" >&2
fi
if [[ -n "${vid50:-}" ]]; then
  sudo ln -sfn "$vid50" /dev/video50
else
  echo "ERROR: no even video on 1-2.3 (sensor fisheye)" >&2
fi
if [[ -n "${vid60:-}" ]]; then
  sudo ln -sfn "$vid60" /dev/video60
else
  echo "ERROR: no even video on 1-1.3 (gripper fisheye)" >&2
fi

echo
echo "==== resulting links ===="
ls -l /dev/video50 /dev/video60 /dev/ttyUSB50 /dev/ttyUSB60 2>&1 || true
echo
echo "Verify AS5047:"
echo "  stty -F /dev/ttyUSB50 460800 raw -echo"
echo "  timeout 0.5 cat /dev/ttyUSB50 | head -c 200"

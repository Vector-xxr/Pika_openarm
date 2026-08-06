#!/bin/bash
set -euo pipefail

sudo tee /etc/udev/rules.d/sensor_serial.rules >/dev/null <<'EOF'
ACTION=="add", KERNELS=="1-1.4:1.0", SUBSYSTEMS=="usb", MODE:="0777", SYMLINK+="ttyUSB50"
EOF
sudo tee /etc/udev/rules.d/gripper_serial.rules >/dev/null <<'EOF'
ACTION=="add", KERNELS=="1-2.4:1.0", SUBSYSTEMS=="usb", MODE:="0777", SYMLINK+="ttyUSB60"
EOF
sudo tee /etc/udev/rules.d/sensor_fisheye.rules >/dev/null <<'EOF'
ACTION=="add", KERNEL=="video*", ATTR{index}=="0", KERNELS=="1-1.3:1.0", SUBSYSTEMS=="usb", MODE:="0777", SYMLINK+="video50"
EOF
sudo tee /etc/udev/rules.d/gripper_fisheye.rules >/dev/null <<'EOF'
ACTION=="add", KERNEL=="video*", ATTR{index}=="0", KERNELS=="1-2.3:1.0", SUBSYSTEMS=="usb", MODE:="0777", SYMLINK+="video60"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty --subsystem-match=video4linux --action=add || true

# Force symlinks for already-present devices (udev often skips).
resolve_capture_on_port() {
  local want="$1"
  local sysdev name idx path
  for sysdev in /sys/class/video4linux/video*; do
    [[ -e "$sysdev" ]] || continue
    name=$(basename "$sysdev")
    case "$name" in video50|video51|video60|video61|video70) continue ;; esac
    idx=$(cat "$sysdev/index" 2>/dev/null || echo "")
    [[ "$idx" == "0" ]] || continue
    path=$(udevadm info -q path -n "/dev/$name" 2>/dev/null || true)
    if [[ "$path" == *"/${want}/"* ]]; then
      printf '%s\n' "$name"
      return 0
    fi
  done
  return 1
}

resolve_tty_on_port() {
  local want="$1"
  local sysdev name path
  for sysdev in /sys/class/tty/ttyUSB*; do
    [[ -e "$sysdev" ]] || continue
    name=$(basename "$sysdev")
    case "$name" in ttyUSB50|ttyUSB51|ttyUSB60|ttyUSB61|ttyUSB70) continue ;; esac
    path=$(udevadm info -q path -n "/dev/$name" 2>/dev/null || true)
    if [[ "$path" == *"/${want}/"* ]]; then
      printf '%s\n' "$name"
      return 0
    fi
  done
  return 1
}

t1=$(resolve_tty_on_port "1-1.4:1.0" || true)
t2=$(resolve_tty_on_port "1-2.4:1.0" || true)
v1=$(resolve_capture_on_port "1-1.3:1.0" || true)
v2=$(resolve_capture_on_port "1-2.3:1.0" || true)
[[ -n "${t1:-}" ]] && sudo ln -sfn "$t1" /dev/ttyUSB50
[[ -n "${t2:-}" ]] && sudo ln -sfn "$t2" /dev/ttyUSB60
[[ -n "${v1:-}" ]] && sudo ln -sfn "$v1" /dev/video50
[[ -n "${v2:-}" ]] && sudo ln -sfn "$v2" /dev/video60

echo "==== resulting links ===="
ls -l /dev/ttyUSB50 /dev/ttyUSB60 /dev/video50 /dev/video60 2>&1 || true

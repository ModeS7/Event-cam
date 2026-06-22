#!/usr/bin/env bash
#
# evk4_doctor.sh -- one-command health check for the Event-cam / EVK4 setup.
# Diagnoses the common stumbles documented in docs/troubleshooting.md:
# ROS env, dependencies, the udev rule, USB enumeration + permissions, stale
# Fast DDS shared-memory debris, and whether the workspace is built + sourced.
#
# Read-only; no sudo needed. Run it any time something misbehaves:
#   ./setup/evk4_doctor.sh
#
# Exits non-zero if a CRITICAL check fails (warnings do not fail it).

# No `set -e`: every check should run and report, not abort on the first failure.
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
fail=0; warn=0
ok()     { printf '  [OK]   %s\n' "$1"; }
warned() { printf '  [WARN] %s\n' "$1"; warn=$((warn + 1)); }
bad()    { printf '  [FAIL] %s\n' "$1"; fail=$((fail + 1)); }
hint()   { printf '         -> %s\n' "$1"; }

echo "evk4 doctor (ROS_DISTRO=$ROS_DISTRO)"

echo
echo "ROS 2 environment:"
if command -v ros2 >/dev/null 2>&1; then
  ok "ros2 CLI on PATH"
else
  bad "ros2 not on PATH"
  hint "source /opt/ros/$ROS_DISTRO/setup.bash (see docs/installation.md)"
fi
if [ -d "/opt/ros/$ROS_DISTRO" ]; then
  ok "/opt/ros/$ROS_DISTRO present"
else
  bad "ROS $ROS_DISTRO not found at /opt/ros/$ROS_DISTRO"
  hint "install ROS 2 $ROS_DISTRO, or set ROS_DISTRO to your distro"
fi

echo
echo "Dependencies (apt):"
if dpkg -s "ros-$ROS_DISTRO-openeb-vendor" >/dev/null 2>&1; then
  ok "openeb-vendor (Metavision SDK open edition) installed"
else
  bad "ros-$ROS_DISTRO-openeb-vendor not installed"
  hint "run setup/install_deps.sh (see docs/installation.md)"
fi
if dpkg -s "ros-$ROS_DISTRO-event-camera-py" >/dev/null 2>&1; then
  ok "event-camera-py (Python decoder) installed"
else
  warned "ros-$ROS_DISTRO-event-camera-py not installed"
  hint "run setup/install_deps.sh"
fi

echo
echo "udev rule:"
if ls /etc/udev/rules.d/88-cyusb.rules >/dev/null 2>&1; then
  ok "88-cyusb.rules installed"
else
  bad "udev rule 88-cyusb.rules missing -- the camera will be root-only"
  hint "copy setup/udev_rules/88-cyusb.rules to /etc/udev/rules.d, reload, replug (installation.md)"
fi

echo
echo "EVK4 USB:"
if command -v lsusb >/dev/null 2>&1; then
  line=$(lsusb | grep -i "04b4:00f5" | head -1)
  if [ -n "$line" ]; then
    ok "EVK4 enumerated: $line"
    bus=$(echo "$line" | sed -E 's/^Bus ([0-9]+) Device ([0-9]+).*/\1/')
    dev=$(echo "$line" | sed -E 's/^Bus ([0-9]+) Device ([0-9]+).*/\2/')
    node="/dev/bus/usb/$bus/$dev"
    perms=$(stat -c '%A' "$node" 2>/dev/null)
    if [ -n "$perms" ] && [ "${perms:7:2}" = "rw" ]; then
      ok "$node is world-readable/writable ($perms)"
    else
      warned "$node is not world-rw (${perms:-?}) -- the udev rule may not have applied"
      hint "sudo udevadm control --reload-rules && sudo udevadm trigger, then replug (troubleshooting.md)"
    fi
  else
    warned "EVK4 (USB 04b4:00f5, Cypress FX3) not found"
    hint "plug the camera into a USB3 port; if already plugged, replug and check the cable (troubleshooting.md)"
  fi
else
  warned "lsusb not available -- cannot check USB enumeration"
fi

echo
echo "Fast DDS shared memory (/dev/shm):"
shm=$(ls /dev/shm 2>/dev/null | grep -cE "fastrtps|sem.fastrtps")
if [ "$shm" -gt 0 ]; then
  if pgrep -f "component_container|driver_node|_node|ros2 " >/dev/null 2>&1; then
    ok "$shm Fast DDS objects present, and ROS nodes are running (normal)"
  else
    warned "$shm Fast DDS objects present but no ROS nodes running -- possibly stale debris"
    hint "if comms are broken after a kill -9 / crash: rm /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* (troubleshooting.md)"
  fi
else
  ok "no stale Fast DDS shared-memory objects"
fi

echo
echo "Workspace / packages:"
if command -v ros2 >/dev/null 2>&1; then
  if ros2 pkg prefix evk4_driver >/dev/null 2>&1; then
    ok "evk4_driver found (this repo's overlay is built + sourced)"
  else
    warned "evk4_driver not found on the ROS package path"
    hint "build ~/ros2_ws and open a new terminal (or 'source ~/.bashrc') -- see docs/installation.md"
  fi
  if ros2 pkg prefix event_camera_renderer >/dev/null 2>&1; then
    ok "event_camera_renderer found"
  else
    warned "event_camera_renderer not found"
    hint "run setup/install_deps.sh (builds the 3rd_party_ws overlay)"
  fi
else
  warned "ros2 not on PATH -- skipping package checks (source the workspace first)"
fi

echo
if [ "$fail" -gt 0 ]; then
  echo "RESULT: $fail critical issue(s), $warn warning(s). See docs/troubleshooting.md."
  exit 1
elif [ "$warn" -gt 0 ]; then
  echo "RESULT: no critical issues, $warn warning(s). See docs/troubleshooting.md if something misbehaves."
else
  echo "RESULT: all checks passed."
fi

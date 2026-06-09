#!/usr/bin/env bash
#
# Install the Event-cam dependencies (Prophesee EVK4 stack) for ROS 2.
#
# Binaries are used where they exist: the driver, the Python decoder, and the
# renderer come from apt when available. On a platform with no renderer binary
# (e.g. some ARM64), the renderer and its decode libraries (event_camera_codecs
# / _msgs) are built from source via `vcs import` into a dedicated 3rd-party
# (underlay) workspace. Finally it installs the vendored udev rule.
#
# This repo (the overlay) is built separately afterwards — see the closing
# message and docs/installation.md.
#
# Usage:
#   export ROS_DISTRO=jazzy            # or humble
#   ./setup/install_deps.sh            # underlay at ~/workspaces/3rd_party_ws
#   ./setup/install_deps.sh /path/ws   # or a custom workspace path
#
# Status: the source/ARM path is not yet hardware-validated. If even the driver
# has no ARM binary, build OpenEB + the driver from source first — see
# docs/installation.md, step 3, ARM64.

set -euo pipefail

if [ -z "${ROS_DISTRO:-}" ]; then
  echo "ERROR: ROS_DISTRO is not set. Run: export ROS_DISTRO=jazzy  (or humble)" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
WS_PATH="${1:-$HOME/workspaces/3rd_party_ws}"
UDEV_SRC="$SCRIPT_DIR/udev_rules"
UDEV_DST="/etc/udev/rules.d"

echo "Event-cam dependency install (ROS_DISTRO=$ROS_DISTRO)"

echo "[1/3] apt: build tools, driver, Python decoder..."
sudo apt update
sudo apt install -y \
  ros-dev-tools \
  ros-"$ROS_DISTRO"-metavision-driver \
  ros-"$ROS_DISTRO"-event-camera-py
sudo rosdep init 2>/dev/null || true
rosdep update

echo "[2/3] renderer: apt if available, else build from source..."
if sudo apt install -y "ros-$ROS_DISTRO-event-camera-renderer"; then
  echo "  installed event_camera_renderer from apt"
else
  echo "  no renderer binary for this platform; building from source in $WS_PATH"
  mkdir -p "$WS_PATH/src"
  cd "$WS_PATH/src"
  [ -d event_camera_renderer ] || \
    git clone https://github.com/ros-event-camera/event_camera_renderer.git
  # pulls event_camera_msgs + event_camera_codecs
  vcs import --input event_camera_renderer/event_camera_renderer.repos .
  cd "$WS_PATH"
  set +u   # ROS setup scripts reference unbound vars
  source /opt/ros/"$ROS_DISTRO"/setup.bash
  set -u
  rosdep install --from-paths src --ignore-src -r -y
  # RelWithDebInfo: the high event rate makes an unoptimized build slow.
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
  SOURCE_LINE="source $WS_PATH/install/setup.bash"
  if ! grep -Fxq "$SOURCE_LINE" "$HOME/.bashrc"; then
    echo "$SOURCE_LINE" >> "$HOME/.bashrc"
    echo "  added underlay sourcing to ~/.bashrc"
  fi
fi

echo "[3/3] udev rule..."
if [ -d "$UDEV_SRC" ]; then
  sudo cp "$UDEV_SRC"/*.rules "$UDEV_DST"/
  sudo udevadm control --reload-rules
  sudo udevadm trigger
  echo "  installed udev rule(s) from $UDEV_SRC — unplug and replug the camera"
else
  echo "  WARNING: $UDEV_SRC not found; skipping udev (camera may be root-only)"
fi

echo
echo "Done. Open a new terminal (or 'source ~/.bashrc'), then build this repo"
echo "as the overlay:"
echo "    cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash"

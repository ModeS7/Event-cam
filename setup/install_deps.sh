#!/usr/bin/env bash
#
# Install the Event-cam dependencies (Prophesee EVK4 stack) for ROS 2.
#
# Layout (matches the AIS-CPS-Lab convention): OpenEB (the Metavision SDK open
# edition) and the Python decoder come from apt, but event_camera_renderer and
# its decode libraries (event_camera_codecs / event_camera_msgs, pulled via the
# renderer's .repos) are built FROM SOURCE into a dedicated 3rd-party (underlay)
# workspace. That keeps the rendering/decoding code a controlled, modifiable
# copy, cleanly separated from the apt system install (/opt/ros) and from your
# own overlay (~/ros2_ws). Our own evk4_driver (built on OpenEB) lives in this
# repo (the overlay) and is built separately afterwards.
#
# Usage:
#   export ROS_DISTRO=jazzy            # or humble
#   ./setup/install_deps.sh            # underlay at ~/workspaces/3rd_party_ws
#   ./setup/install_deps.sh /path/ws   # or a custom workspace path

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

echo "[1/4] apt: build tools, OpenEB (Metavision SDK), Python decoder..."
sudo apt update
sudo apt install -y \
  ros-dev-tools \
  ros-"$ROS_DISTRO"-openeb-vendor \
  ros-"$ROS_DISTRO"-event-camera-py
sudo rosdep init 2>/dev/null || true
rosdep update

echo "[2/4] source build: renderer + decode libs -> $WS_PATH ..."
mkdir -p "$WS_PATH/src"
cd "$WS_PATH/src"
[ -d event_camera_renderer ] || \
  git clone https://github.com/ros-event-camera/event_camera_renderer.git
# pulls event_camera_msgs + event_camera_codecs
vcs import --input event_camera_renderer/event_camera_renderer.repos .
# Apply our renderer patch (bounds the pending-frame backlog; without it the
# viewer replays seconds of stale frames after quiet periods). Idempotent:
# skipped when already applied. Upstream PR pending.
PATCH="$SCRIPT_DIR/patches/event_camera_renderer-backlog-cap.patch"
if git -C event_camera_renderer apply --reverse --check "$PATCH" 2>/dev/null; then
  echo "renderer patch already applied"
else
  git -C event_camera_renderer apply "$PATCH"
  echo "renderer patch applied"
fi
cd "$WS_PATH"
set +u   # ROS setup scripts reference unbound vars
source /opt/ros/"$ROS_DISTRO"/setup.bash
set -u
rosdep install --from-paths src --ignore-src -r -y
# RelWithDebInfo: the high event rate makes an unoptimized build slow.
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

echo "[3/4] source the deps workspace from ~/.bashrc (and /etc/skel)..."
# Guarded so a new terminal opened before the workspace is built (or after
# it is removed) does not error with "No such file or directory".
SOURCE_LINE="[ -f $WS_PATH/install/setup.bash ] && source $WS_PATH/install/setup.bash"
if ! grep -Fxq "$SOURCE_LINE" "$HOME/.bashrc"; then
  echo "$SOURCE_LINE" >> "$HOME/.bashrc"
  echo "  added to ~/.bashrc"
fi
if [ -f /etc/skel/.bashrc ] && ! sudo grep -Fxq "$SOURCE_LINE" /etc/skel/.bashrc; then
  echo "$SOURCE_LINE" | sudo tee -a /etc/skel/.bashrc >/dev/null
  echo "  added to /etc/skel/.bashrc (future users)"
fi

echo "[4/4] udev rule..."
if [ -d "$UDEV_SRC" ]; then
  sudo cp "$UDEV_SRC"/*.rules "$UDEV_DST"/
  sudo udevadm control --reload-rules
  sudo udevadm trigger
  echo "  installed udev rule(s) from $UDEV_SRC - unplug and replug the camera"
else
  echo "  WARNING: $UDEV_SRC not found; skipping udev (camera may be root-only)"
fi

echo
echo "Done. Open a new terminal (or 'source ~/.bashrc'), then build this repo"
echo "as the overlay:"
echo "    cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash"

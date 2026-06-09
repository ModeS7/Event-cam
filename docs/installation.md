# Installation

Run these on the machine the camera is plugged into (lab PC, Raspberry Pi, or
any host). The dev machine where you write code needs only `git` and an editor.

**Only the dependency step differs by platform** — and it differs by **CPU
architecture** (x86 vs ARM), because that decides whether the OpenEB SDK comes
from apt or a source build. The board itself (NUC, Raspberry Pi, Jetson, …)
doesn't matter beyond that.

| Tier | Architecture / OS | Status |
|---|---|---|
| 1 | x86_64 · Ubuntu 24.04 · Jazzy | validated on hardware (2026-06-05) |
| 2 | x86_64 · Ubuntu 22.04 · Humble | expected — same steps, untested |
| 3 | ARM64 SBC (Raspberry Pi, Jetson, …) · Jazzy or Humble | experimental — may need source-built OpenEB |
| — | Ubuntu < 22.04 (e.g. original Jetson Nano @ 18.04) | unsupported — ROS 2 is EOL there |

Set your distro once so the commands copy-paste cleanly:

```bash
export ROS_DISTRO=jazzy      # jazzy (Tier 1) or humble (Tier 2/3)
```

## 1. Prerequisites

- The Ubuntu + ROS 2 version for your tier, installed
  ([official guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)).
  If a `ros-...` package is "unable to locate", the ROS 2 apt repo is missing —
  see [troubleshooting.md](troubleshooting.md#e-unable-to-locate-package-ros-jazzy-).
- **On a Raspberry Pi / SBC: flash Ubuntu (24.04 LTS, 64-bit), NOT Raspberry
  Pi OS.** ROS 2 ships apt binaries only for Ubuntu; Raspberry Pi OS (Debian)
  has no `ros-...` packages. In the Raspberry Pi Imager pick *Other
  general-purpose OS → Ubuntu → Ubuntu Server/Desktop 24.04 LTS (64-bit)*.
  After first boot, ensure `/etc/apt/sources.list.d/ubuntu.sources` lists
  `Suites: noble noble-updates noble-backports` (Pi images sometimes ship with
  only base `noble`, which breaks `ros-dev-tools`).
- A USB 3.x port and cable — the EVK4 can exceed what USB 2 carries.

## 2. Get this repo

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/ModeS7/Event-cam.git
```

## 3. Install the dependencies

**Quick path** — the script installs the camera stack and the udev rule
(builds the renderer from source where needed, with optimized flags):

```bash
~/ros2_ws/src/Event-cam/setup/install_deps.sh
```

Everything below explains what it does and is the manual / fallback path.

### What the script does — manual equivalent

**x86_64 (Tiers 1 & 2):** all binaries exist; OpenEB rides along via
`openeb_vendor`.

```bash
sudo apt install \
  ros-$ROS_DISTRO-metavision-driver \
  ros-$ROS_DISTRO-event-camera-renderer \
  ros-$ROS_DISTRO-event-camera-py
```

**ARM64 (Tier 3, experimental — not yet run on hardware):** ROS 2 publishes
ARM64 packages, so the driver and `event_camera_py` may install straight from
apt (the script tries this). The renderer is built from source into a
dedicated **deps (underlay) workspace** so it isn't recompiled when you edit
your own code — using `vcs import` to pull its decode libraries, and
`RelWithDebInfo` because the high event rate makes an unoptimized build slow:

```bash
mkdir -p ~/workspaces/3rd_party_ws/src && cd ~/workspaces/3rd_party_ws/src
git clone https://github.com/ros-event-camera/event_camera_renderer.git
vcs import --input event_camera_renderer/event_camera_renderer.repos .  # pulls msgs + codecs
cd ~/workspaces/3rd_party_ws
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

If apt has **no** ARM64 driver binary either, build OpenEB from source first
([Prophesee guide](https://docs.prophesee.ai/stable/installation/linux_openeb.html);
ARM is untested upstream, so budget time), then build `metavision_driver` from
source in the same deps workspace. See the **Workspaces** note at the end.

### udev rule (all platforms)

The rule is vendored at `setup/udev_rules/88-cyusb.rules` (the script installs
it). Manually:

```bash
sudo cp ~/ros2_ws/src/Event-cam/setup/udev_rules/88-cyusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Then **unplug and replug** the camera. The rule grants all users access to any
Cypress `04b4` device; its `cy_renumerate.sh` clause logs a harmless warning.

## 4. Build this repo (the overlay)

```bash
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/workspaces/3rd_party_ws/install/setup.bash   # ARM source build only
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

On x86 (Tiers 1–2) skip the deps-workspace `source` line — the dependencies
come from apt under `/opt/ros`, which is already the underlay.

## 5. Smoke test

```bash
lsusb | grep -i 04b4                    # camera enumerated? expect 04b4:00f5
ros2 launch evk4_bringup evk4.launch.py
```

In a second terminal (`source ~/ros2_ws/install/setup.bash` first):

```bash
ros2 topic hz /event_camera/events      # rate appears when the scene changes
ros2 run rqt_image_view rqt_image_view /event_camera/image_raw
```

Wave a hand in front of the camera — event cameras only produce data on
brightness change. If anything fails, see
[troubleshooting.md](troubleshooting.md).

## Workspaces (underlay / overlay)

ROS 2 layers workspaces: each one you `source` stacks on top of the previous
(an *overlay*) and sees everything beneath it (the *underlay*). Keep
rarely-changing dependencies in a lower layer so editing your own code only
rebuilds your own packages.

- **x86 (Tiers 1–2):** two layers, automatic. `/opt/ros/$ROS_DISTRO` (apt deps)
  is the underlay; `~/ros2_ws` (this repo) is the overlay.
- **ARM source build (Tier 3):** three layers. `/opt/ros/$ROS_DISTRO` →
  `~/workspaces/3rd_party_ws` (source-built renderer etc.) → `~/ros2_ws` (this
  repo). Build the deps workspace once; thereafter `colcon build` in
  `~/ros2_ws` recompiles only our small packages, not the heavy upstream tree.

Source order is bottom-up; put these in `~/.bashrc` so every terminal is ready
(the script adds the deps-workspace line for you):

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/workspaces/3rd_party_ws/install/setup.bash    # ARM source build only
source ~/ros2_ws/install/setup.bash
```

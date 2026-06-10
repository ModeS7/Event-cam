# Installation

Run these on the machine the camera is plugged into (lab PC, Raspberry Pi, or
any host). The dev machine where you write code needs only `git` and an editor.

**The install steps are the same on every platform** — OpenEB + the Python
decoder from apt (arm64 and x86 binaries both exist), the renderer from
source, and our own driver built from this repo. The board (NUC, Raspberry
Pi, Jetson, …) doesn't change the procedure; the table below just records
where it's been validated.

| Platform | Status |
|---|---|
| x86_64 · Ubuntu 24.04 · Jazzy | validated on hardware (2026-06-05) |
| x86_64 · Ubuntu 22.04 · Humble | expected — untested |
| ARM64 · Raspberry Pi 5 · Ubuntu 24.04 · Jazzy | validated on hardware (2026-06-09) |
| Other ARM64 SBCs / Humble | expected |
| Ubuntu < 22.04 (e.g. original Jetson Nano @ 18.04) | unsupported — ROS 2 is EOL there |

Set your distro once so the commands copy-paste cleanly:

```bash
export ROS_DISTRO=jazzy      # jazzy (Ubuntu 24.04) or humble (Ubuntu 22.04)
```

## 1. Prerequisites

- The Ubuntu + ROS 2 version for your platform, installed
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

**Quick path** — the script installs everything and the udev rule:

```bash
~/ros2_ws/src/Event-cam/setup/install_deps.sh
```

Everything below explains what it does and is the manual equivalent. It is the
**same on every platform** (OpenEB + decoder are arm64/x86 binaries; the
renderer is built from source either way). Our own `evk4_driver` is built from
this repo (step 4) directly on OpenEB — there is no separate driver to install:

```bash
# 1. OpenEB (the Metavision SDK open edition) + Python decoder from apt
sudo apt install ros-dev-tools \
  ros-$ROS_DISTRO-openeb-vendor \
  ros-$ROS_DISTRO-event-camera-py
sudo rosdep init 2>/dev/null || true; rosdep update

# 2. renderer + its decode libs (event_camera_codecs/_msgs via vcs import)
#    from SOURCE into a dedicated 3rd-party workspace, kept separate so it is
#    not recompiled when you edit your own code; RelWithDebInfo because the
#    high event rate makes an unoptimized build slow
mkdir -p ~/workspaces/3rd_party_ws/src && cd ~/workspaces/3rd_party_ws/src
git clone https://github.com/ros-event-camera/event_camera_renderer.git
vcs import --input event_camera_renderer/event_camera_renderer.repos .
cd ~/workspaces/3rd_party_ws
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# 3. source it (the script also appends this to ~/.bashrc and /etc/skel)
echo "source ~/workspaces/3rd_party_ws/install/setup.bash" >> ~/.bashrc
```

(If you are on a platform where `openeb_vendor` has no apt binary, build OpenEB
from source first
[[Prophesee guide]](https://docs.prophesee.ai/stable/installation/linux_openeb.html);
our `evk4_driver` then builds against it like any other underlay.) See the
**Workspaces** note at the end.

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
source ~/workspaces/3rd_party_ws/install/setup.bash   # the source-built deps
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

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
rebuilds your own packages. **Three layers, on every platform:**

- `/opt/ros/$ROS_DISTRO` — ROS + apt OpenEB/decoder (underlay 0)
- `~/workspaces/3rd_party_ws` — source-built renderer + `event_camera_codecs`/
  `_msgs`, deliberately separated so you can modify the rendering/decoding code
  without touching the system install or your own packages (underlay 1)
- `~/ros2_ws` — this repo (overlay)

Build the deps workspace once; thereafter `colcon build` in `~/ros2_ws`
recompiles only our small packages, not the renderer tree. Source order is
bottom-up; the script appends these to `~/.bashrc` (and `/etc/skel/.bashrc` for
future users):

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/workspaces/3rd_party_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
```

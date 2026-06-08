# Installation

Run these on the machine the camera is plugged into (lab PC or drone). The dev
machine where you write code needs only `git` and an editor.

**Only step 2 differs** — and it differs by **CPU architecture** (x86 vs ARM),
because that decides whether the OpenEB SDK comes from apt or a source build.
The board itself (NUC, Raspberry Pi, Jetson, …) doesn't matter beyond that.
Pick your row, do step 2, then steps 3–6 are identical for everyone.

| Tier | Architecture / OS | Status |
|---|---|---|
| 1 | x86_64 · Ubuntu 24.04 · Jazzy | validated on hardware (2026-06-05) |
| 2 | x86_64 · Ubuntu 22.04 · Humble | expected — same steps, untested |
| 3 | ARM64 SBC (Raspberry Pi, Jetson, …) · Jazzy or Humble | experimental — may need source-built OpenEB |
| — | Ubuntu < 22.04 (e.g. original Jetson Nano @ 18.04) | unsupported — ROS 2 is EOL there |

Set your distro once so the commands below copy-paste cleanly:

```bash
export ROS_DISTRO=jazzy      # jazzy (Tier 1) or humble (Tier 2/3)
```

## 1. Prerequisites

- The Ubuntu + ROS 2 version for your tier, installed
  ([official guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)).
- A USB 3.x port and cable — the EVK4 can exceed what USB 2 carries.

## 2. Install the camera stack

### x86_64 (Tiers 1 & 2)

You can skip this — `rosdep` in step 5 installs the whole stack automatically.
To pre-install it by hand instead:

```bash
sudo apt install \
  ros-$ROS_DISTRO-metavision-driver \
  ros-$ROS_DISTRO-event-camera-renderer \
  ros-$ROS_DISTRO-event-camera-py
```

OpenEB is bundled via `openeb_vendor` — no separate Metavision SDK. (Validated
against driver 3.0.0; check yours with `apt policy ros-$ROS_DISTRO-metavision-driver`.)

### ARM64 SBC — Raspberry Pi, Jetson, any ARM board (Tier 3, experimental)

ROS 2 publishes ARM64 packages, so first just **try the x86 apt command above**
— it may work. The open question is OpenEB: whether `metavision_driver`'s ARM64
binary is built on the ROS farm is unconfirmed. If apt can't find it, build
OpenEB from source, then build the driver against it:

1. Build and install OpenEB from source
   ([Prophesee guide](https://docs.prophesee.ai/stable/installation/linux_openeb.html)).
   Prophesee notes ARM compilation is untested, so budget time here.
2. Clone `metavision_driver` and the `event_camera_*` repos into your
   workspace `src/`; the driver links against the OpenEB you just installed.
3. Continue to step 5 — `rosdep`/`colcon` build the rest normally.

None of this is verified on hardware yet. Also weigh the board: a high event
rate over USB plus your other workloads can outrun a small SBC's CPU/USB.

## 3. udev rule (one-time, all platforms)

The EVK4 enumerates as a Cypress FX3 device (vendor `04b4`). Without this rule
only root can open it and the driver won't find the camera — and apt does
**not** install it. Add the rule from OpenEB:

```bash
sudo wget -O /etc/udev/rules.d/88-cyusb.rules \
  https://raw.githubusercontent.com/prophesee-ai/openeb/main/hal_psee_plugins/resources/rules/88-cyusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Then **unplug and replug** the camera. (The rule grants all users access to any
Cypress `04b4` device; its `cy_renumerate.sh` clause logs a harmless warning.)

## 4. Verify the camera is visible

```bash
lsusb | grep -i 04b4
# Bus 002 Device 003: ID 04b4:00f5 Cypress Semiconductor Corp. ...
```

## 5. Build this repo

`colcon` and `rosdep` come from `ros-dev-tools` (not in `…-desktop`):

```bash
sudo apt install ros-dev-tools
sudo rosdep init && rosdep update    # first time only; init is harmless if repeated
```

Build. `rosdep install` is the portable step — it reads each `package.xml` and
pulls the right dependencies for your distro (this is what installs the camera
stack on Tiers 1–2):

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/ModeS7/Event-cam.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
```

## 6. Smoke test

```bash
ros2 launch evk4_bringup evk4.launch.py
```

In a second terminal (`source ~/ros2_ws/install/setup.bash` first):

```bash
ros2 topic hz /event_camera/events     # rate appears when the scene changes
ros2 run rqt_image_view rqt_image_view /event_camera/image_raw
```

Wave a hand in front of the camera — event cameras only produce data on
brightness change. If anything fails, see
[troubleshooting.md](troubleshooting.md).

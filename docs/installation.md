# Installation

Target platform: **Ubuntu 24.04 + ROS 2 Jazzy** with a Prophesee **EVK4 HD**
(Sony IMX636). All commands run on the machine the camera is plugged into.

## 1. Prerequisites

- Ubuntu 24.04 with ROS 2 Jazzy installed
  ([official guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html))
  — if step 2 below fails with `Unable to locate package`, the ROS 2 apt
  repository is missing; see
  [troubleshooting.md](troubleshooting.md#e-unable-to-locate-package-ros-jazzy-)
- A **USB 3.x** port and cable — the EVK4 can exceed what USB 2 can carry

## 2. Install the driver stack (apt)

```bash
sudo apt install \
  ros-jazzy-metavision-driver \
  ros-jazzy-event-camera-renderer \
  ros-jazzy-event-camera-py \
  ros-jazzy-diagnostic-updater
```

| Package | Purpose | Version (2026-06) |
|---|---|---|
| `ros-jazzy-metavision-driver` | Camera driver; bundles OpenEB via `openeb_vendor` | 3.0.0 |
| `ros-jazzy-event-camera-renderer` | Renders events to `sensor_msgs/Image` | 3.0.0 |
| `ros-jazzy-event-camera-py` | Python decoder used by `evk4_examples` | 3.0.0 |
| `ros-jazzy-diagnostic-updater` | Diagnostics support for `evk4_diagnostics` | — |

No separate Metavision SDK installation is needed — OpenEB is installed
under `/opt/ros/jazzy` as a dependency. Check your installed version with
`apt policy ros-jazzy-metavision-driver`.

Optional CLI utilities (echo/perf/bag conversion):
`sudo apt install ros-jazzy-event-camera-tools`

## 3. udev rule (one-time, required)

The EVK4 enumerates as a Cypress FX3 USB device (vendor ID `04b4`). Without
a udev rule only root can open it, and the driver will fail to find the
camera. **The apt packages do not install this rule.** Install the rule
shipped with OpenEB:

```bash
sudo wget -O /etc/udev/rules.d/88-cyusb.rules \
  https://raw.githubusercontent.com/prophesee-ai/openeb/main/hal_psee_plugins/resources/rules/88-cyusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Then **unplug and replug** the camera.

Notes:

- The rule grants all users read/write access (`MODE="666"`) to any Cypress
  (`04b4`) USB device.
- The rule's `RUN+=".../cy_renumerate.sh"` clause refers to a script that
  only exists with Cypress' own SDK; udev logs a harmless warning about it.

## 4. Verify the camera is visible

```bash
lsusb | grep -i 04b4
# expected (bus/device numbers vary):
# Bus 002 Device 003: ID 04b4:00f5 Cypress Semiconductor Corp. ...
```

## 5. Build this repo

`colcon` and `rosdep` ship in `ros-dev-tools`, **not** in `ros-jazzy-desktop`:

```bash
sudo apt install ros-dev-tools
```

If rosdep has never been used on this machine, initialize it once
(`init` errors harmlessly if it was already done):

```bash
sudo rosdep init
rosdep update
```

Then build:

```bash
source /opt/ros/jazzy/setup.bash    # or add to ~/.bashrc
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/ModeS7/Event-cam.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y   # fails loudly if deps missing
colcon build
source install/setup.bash
```

## 6. Smoke test

```bash
ros2 launch evk4_bringup evk4.launch.py
```

In a second terminal (remember to `source ~/ros2_ws/install/setup.bash`):

```bash
ros2 topic hz /event_camera/events        # messages appear when the scene changes
ros2 run rqt_image_view rqt_image_view /event_camera/image_raw
```

Wave a hand in front of the camera — event cameras only produce data when
brightness changes. If anything fails, see
[troubleshooting.md](troubleshooting.md).

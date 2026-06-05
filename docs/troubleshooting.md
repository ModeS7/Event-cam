# Troubleshooting

Symptoms are ordered roughly by how often they occur.

## `E: Unable to locate package ros-jazzy-...`

The ROS 2 apt repository is not configured on this machine — `ros-jazzy-*`
packages come from `packages.ros.org`, not stock Ubuntu. Enable it (from the
[official guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)):

```bash
sudo apt install software-properties-common curl -y
sudo add-apt-repository universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
```

If ROS 2 itself is missing too (`ls /opt/ros/` is empty), install it first:
`sudo apt install ros-jazzy-desktop ros-dev-tools` (desktop includes
`rqt_image_view`, used below for visualization; `ros-dev-tools` provides
`colcon` and `rosdep` for the build step).

## Launch aborts: `required package '...' is not installed`

Our launch file checks dependencies up front. Install them:

```bash
sudo apt install ros-jazzy-metavision-driver ros-jazzy-event-camera-renderer
```

## Driver starts but finds no camera

1. Is the camera enumerated at all?
   ```bash
   lsusb | grep -i 04b4
   ```
   Nothing → bad cable/port. Use USB 3.x, try another port, check `dmesg -w`
   while replugging.
2. Permissions (the most common cause). With the device from `lsusb`
   (`Bus 002 Device 003` → `/dev/bus/usb/002/003`):
   ```bash
   ls -l /dev/bus/usb/002/003   # must be rw for everyone: crw-rw-rw-
   ```
   If not, the udev rule is missing — see
   [installation.md §3](installation.md#3-udev-rule-one-time-required),
   then replug.
3. Another process may hold the camera (e.g. a second driver instance or
   Metavision Studio). Only one consumer can open it.

## `cannot open default on attempt 1, retrying ...` then `giving up!`

The camera exists but can't be opened — almost always it's held by another
process. A previous driver instance that didn't shut down cleanly (the
driver can hang on Ctrl-C) is the usual culprit:

```bash
pgrep -af driver_node      # any leftover instance?
pkill -9 -f driver_node
```

If it still fails, unplug/replug the camera to reset the USB handle, confirm
with `lsusb | grep -i 04b4`, and launch again.

## Service call hangs: `waiting for service to become available...`

The `save_biases` / `save_settings` / `dump_statistics` services live inside
the driver node — they only exist **while the camera launch is running**.
Call them from a second terminal; the output appears in the launch
terminal's log.

## Topic exists but `ros2 topic hz /event_camera/events` shows nothing

Event cameras only produce output when **brightness changes**. A static
scene under constant light is (almost) silent — this is correct behavior,
not a fault. Wave a hand in front of the lens and check again. Also check
the lens cap and the lens aperture/focus ring.

## My own subscriber receives nothing (but `ros2 topic hz` works)

QoS mismatch — the most common student bug. The driver publishes
**best-effort**; a subscriber with default (reliable) QoS never matches.

- Python: `create_subscription(..., rclpy.qos.qos_profile_sensor_data)`
- C++: `create_subscription(..., rclcpp::SensorDataQoS())`

## `rqt_image_view` image is black or frozen

- No motion in the scene → no events → nothing to render (see above).
- Was the camera launched with `viz:=false`? Check
  `ros2 topic list | grep image_raw`.

## Very high CPU load or dropped messages

In busy scenes the EVK4 can emit hundreds of Mev/s. In
`evk4_bringup/config/evk4_params.yaml`:

- Enable on-sensor event rate control: `erc_mode: enabled` and set
  `erc_rate`.
- Increase `event_message_time_threshold` (fewer, larger messages).
- Set `use_multithreading: true`.

Check the actual rates with
`ros2 service call /event_camera/dump_statistics std_srvs/srv/Trigger` or
`ros2 run evk4_examples event_rate`.

## `ModuleNotFoundError: No module named 'event_camera_py'`

```bash
sudo apt install ros-jazzy-event-camera-py
```

## `Package 'evk4_bringup' not found`

The workspace isn't sourced in this terminal:

```bash
source ~/ros2_ws/install/setup.bash
```

(Or the build failed — rerun `colcon build` and read its output.)

## Multiple cameras

Select by serial: `ros2 launch evk4_bringup evk4.launch.py serial:=<S/N>`.
The serial is printed in the driver's startup log, or query the device with
the camera unplugged from other consumers.

# Troubleshooting

Symptoms are ordered roughly by how often they occur.

## `E: Unable to locate package ros-jazzy-...`

The ROS 2 apt repository is not configured on this machine — `ros-<distro>-*`
packages (`ros-jazzy-*`, `ros-humble-*`, …) come from `packages.ros.org`, not
stock Ubuntu. Enable it (from the
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
`sudo apt install ros-$ROS_DISTRO-desktop ros-dev-tools` (desktop includes
`rqt_image_view` for visualization; `ros-dev-tools` provides `colcon` and
`rosdep` for the build step).

## Launch aborts: `required package '...' is not installed`

Our launch file checks dependencies up front and prints the exact apt name
for **your** ROS distro (it derives it from `$ROS_DISTRO`). Install what the
message names, e.g. on a binary platform:

```bash
sudo apt install ros-$ROS_DISTRO-metavision-driver ros-$ROS_DISTRO-event-camera-renderer
```

On ARM64 (Tier 3) the binary may not exist — build from source if apt can't
find it (see [installation.md](installation.md), step 2, ARM64).

## Driver starts but finds no camera

1. Is the camera enumerated at all?
   ```bash
   lsusb | grep -i 04b4
   ```
   Nothing → bad cable/port. Use USB 3.x, try another port, check
   `sudo dmesg -w` while replugging.
2. Permissions (the most common cause). With the device from `lsusb`
   (`Bus 002 Device 003` → `/dev/bus/usb/002/003`):
   ```bash
   ls -l /dev/bus/usb/002/003   # must be rw for everyone: crw-rw-rw-
   ```
   If not, the udev rule is missing — see
   [installation.md](installation.md#udev-rule-all-platforms) (step 3, udev
   rule), then replug.
3. Another process may hold the camera (e.g. a second driver instance or
   Metavision Studio). Only one consumer can open it.

## `cannot open default on attempt 1, retrying ...` then `giving up!`

The camera exists but can't be opened — almost always it's held by another
process. A previous launch that didn't shut down cleanly is the usual
culprit (our launch runs the driver inside a `component_container_isolated`
process):

```bash
pgrep -af component_container      # any leftover instance?
pkill -9 -f component_container
```

If it still fails, unplug/replug the camera to reset the USB handle, confirm
with `lsusb | grep -i 04b4`, and launch again.

## Service call hangs: `waiting for service to become available...`

Two causes:

1. The `save_biases` / `save_settings` services live inside the driver node —
   they only exist **while the camera launch is running**. Call them from a
   second terminal.
2. The service doesn't exist in your driver version. Notably
   `dump_statistics` appears in the upstream master-branch docs but is NOT in
   the released 3.0.0 apt package. Check what actually exists with
   `ros2 node info /event_camera` (Service Servers section).

## `ros2 topic list` / `ros2 node list` show things that aren't running

The ROS 2 CLI daemon caches the node graph and can keep showing topics and
nodes for a while after their processes died — which makes a dead launch
look alive. Don't trust the list when debugging; verify the process exists
(`pgrep -af component_container`) or reset the cache first:

```bash
ros2 daemon stop   # it restarts automatically on the next ros2 command
ros2 node list     # now reflects reality
```

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

Check the actual rates in the statistics line the driver prints every
second in the launch terminal (`bw in: ... msgs/s in: ...`), or with
`ros2 run evk4_examples event_rate`.

## `ModuleNotFoundError: No module named 'event_camera_py'`

```bash
sudo apt install ros-$ROS_DISTRO-event-camera-py
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

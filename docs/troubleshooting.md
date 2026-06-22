# Troubleshooting

Symptoms are ordered roughly by how often they occur.

## Stream is seconds behind, frames smushed together, high CPU, or topics silently empty

If the pipeline degrades badly — image lagging tens of seconds behind reality,
multiple time slices blended into one frame, node CPU far above normal,
or some subscribers receiving nothing while others
work — and **restarting the nodes does not help**, the cause is usually stale
Fast DDS shared-memory state from a killed ROS process. Force-killing nodes
(`kill -9`, crashes) leaves transport files and port-mutex semaphores behind in
`/dev/shm`, and they poison communication for every later process.

Recovery:

```bash
pkill -f ros2; pkill -f component_container; sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/fast_datasharing*
ros2 daemon stop
# then launch again as usual
```

Prevention: stop nodes with Ctrl+C (SIGINT), never `kill -9`. (Hard-killing
the driver can also wedge the camera's USB controller — see the replug note
below.)

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

## Launch aborts: `required package '...' is not found`

The launch file checks its dependencies up front. The `evk4_*` packages
are built from this repo, so a "not found" there means you still need to build:

```bash
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

OpenEB comes from apt; the renderer is source-built (see
[installation.md](installation.md)):

```bash
sudo apt install ros-$ROS_DISTRO-openeb-vendor
```

## Edges look thick and soft, fine detail is mush

Check the **lens focus** before blaming biases or filters — defocus is easy
to miss on an event camera because there is no static image to judge by, and
it quietly degrades everything downstream (calibration accuracy in
particular). Aim at a flickering high-contrast target (e.g. the blinking
circle grid, <https://modes7.github.io/Event-cam/circle_grid.html>) at
your working distance, watch
`image_raw`, and turn the focus ring until the dots are as small and crisp
as possible (full procedure + aperture tip: tuning.md, "The lens: aperture and focus").

## Driver starts but finds no camera

1. Is the camera enumerated at all?
   ```bash
   lsusb | grep -i 04b4
   ```
   Nothing → bad cable/port, or the camera's USB controller is wedged from a
   hard-killed driver (`kill -9` mid-stream can knock the EVK4 off the bus
   entirely — always stop the driver with Ctrl+C). Unplug and replug the
   camera; use USB 3.x, try another port, check `sudo dmesg -w` while
   replugging.
2. Permissions (the most common cause). With the device from `lsusb`
   (`Bus 002 Device 003` → `/dev/bus/usb/002/003`):
   ```bash
   ls -l /dev/bus/usb/002/003   # must be rw for everyone: crw-rw-rw-
   ```
   If not, the udev rule is missing — see
   [installation.md](installation.md#udev-rule-all-platforms), then replug.
3. Another process may hold the camera (e.g. a second driver instance or
   Metavision Studio). Only one consumer can open it.

## `failed to open camera: ... Camera not found` (but `lsusb` shows it)

The camera exists but can't be opened — almost always it's held by another
process. A previous launch that didn't shut down cleanly is the usual
culprit (the launch runs the driver inside a `component_container_isolated`
process):

```bash
pgrep -af component_container      # any leftover instance?
pkill -INT -f component_container  # graceful stop (never -9: see the
                                   # shared-memory section above)
```

If it still fails, unplug/replug the camera to reset the USB handle, confirm
with `lsusb | grep -i 04b4`, and launch again.

## Service call hangs: `waiting for service to become available...`

The `save_settings` service lives inside the driver node — it only exists
**while the camera launch is running**. Call it from a second terminal. List what actually exists with
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

## Very high CPU load, dropped messages, or frame rate collapsing

Event-pipeline CPU scales with the event rate, and in busy scenes the EVK4
can emit tens of Mev/s — more than small boards can process with the full
display pipeline running. Cap the rate on the sensor: `erc_mode: enabled` +
`erc_rate` in the driver params (the shipped
`evk4_params_recommended.yaml` does this; see
[tuning.md](tuning.md)). Also close viewers/rectification you are not
using, and raise `event_message_time_threshold` for fewer, larger messages.

Check actual rates in the statistics line the driver prints every second in
the launch terminal (`<msgs/s>, <MB/s> (queue 0)`), or with
`ros2 run evk4_examples event_rate`.

## Stream feels laggy even though the event rate is low

If the view trails reality while the driver's statistics line shows a
modest rate, the machine is overloaded — not the pipeline. Check `uptime`:
a load average above your core count means everything downstream of the
topics (viewers, the desktop compositor) runs late. Non-ROS load counts
too: a browser showing the blinking calibration grid is a fullscreen
canvas repainting several times a second, and on a small board that alone
makes the whole view feel laggy. Close what you are not using; while
calibrating, show the blinking grid on a **different device** (a laptop,
or a monitor driven by another machine) instead of the camera host's own
screen — see [calibration.md](calibration.md).

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

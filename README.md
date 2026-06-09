# Event-cam — Plug-and-play ROS 2 setup for the Prophesee EVK4

ROS 2 launch files, configuration, and example code for getting a Prophesee
EVK4 event camera publishing events with minimal setup. This repo **wraps**
the community [metavision_driver](https://github.com/ros-event-camera/metavision_driver)
— it contains no driver code of its own, only launch/config glue,
example consumers, and documentation.

> **Status:** core path validated end-to-end on camera hardware
> (2026-06-05): install, build, live events, visualization, recording,
> playback, and both example consumers (Python + C++). The composed example
> launch (`event_rate_composed.launch.py`) is written and build-checked but
> not yet exercised on hardware.

## Supported platforms

Camera is always a Prophesee EVK4 HD (Sony IMX636, EVT3). The install steps are
the same on every platform (see [docs/installation.md](docs/installation.md));
the table just records where it's been validated:

| Platform | Status |
|---|---|
| x86_64 · Ubuntu 24.04 · Jazzy | validated on hardware |
| ARM64 · Raspberry Pi 5 · Ubuntu 24.04 · Jazzy | validated on hardware |
| x86_64 · Ubuntu 22.04 · Humble | expected, untested |
| Other ARM64 SBCs / Humble | expected |

## Quickstart

The quickstart below is the **Tier 1** (Jazzy / x86_64) path. For Humble or
any ARM64 board, follow [docs/installation.md](docs/installation.md) instead.

```bash
# 1. Install the driver stack (bundles OpenEB — no separate Metavision SDK)
sudo apt install ros-jazzy-metavision-driver ros-jazzy-event-camera-renderer \
                 ros-jazzy-event-camera-py

# 2. Clone into a colcon workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/ModeS7/Event-cam.git

# 3. Install the udev rule (one-time, vendored in the repo), then replug
sudo cp Event-cam/setup/udev_rules/88-cyusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# 4. Build and run (wave a hand — event cameras need motion!)
cd ~/ros2_ws && colcon build && source install/setup.bash
ros2 launch evk4_bringup evk4.launch.py
ros2 run rqt_image_view rqt_image_view /event_camera/image_raw
```

On ARM (Raspberry Pi, Jetson) or for a one-command dependency install, run
`Event-cam/setup/install_deps.sh` instead of steps 1+3 — see
[docs/installation.md](docs/installation.md).

![Rendered event stream in rqt_image_view](docs/images/rqt_image_view.png)

*Live EVK4 event stream rendered to an image (camera panning across the
lab): blue = ON events (brightness increased), red = OFF events
(brightness decreased).*

Full instructions: [docs/installation.md](docs/installation.md).

## Topic contract

`evk4.launch.py` brings up the camera under the `/event_camera` namespace:

| Topic | Type | When |
|---|---|---|
| `/event_camera/events` | `event_camera_msgs/msg/EventPacket` (EVT3) | always |
| `/event_camera/image_raw` | `sensor_msgs/msg/Image` (25 fps render) | `viz:=true` (default) |
| `/event_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | `calibration_url` set |

Raw events are the processing contract — decode them with
[event_camera_py](https://github.com/ros-event-camera/event_camera_py) (Python)
or [event_camera_codecs](https://github.com/ros-event-camera/event_camera_codecs) (C++).
The image topic exists for instant visualization (`rqt_image_view`); disable it
with `viz:=false`. **Note:** events are published best-effort — subscribers
need sensor-data QoS (see [docs/usage.md](docs/usage.md)).

## Repository layout

| Path | Purpose |
|---|---|
| `evk4_bringup/` | Launch file, driver params, bias + calibration configs, `camera_info` helper |
| `evk4_examples/` | Example Python subscriber (`ros2 run evk4_examples event_rate`) |
| `evk4_examples_cpp/` | Same example in C++, as a composable component |
| `evk4_calibration/` | Guided intrinsic calibrator (`ros2 run evk4_calibration calibrate`) |
| `setup/` | `install_deps.sh` (one-command dependency setup) + vendored udev rule |
| `docs/` | Installation, usage, troubleshooting |

## Documentation

- [docs/installation.md](docs/installation.md) — apt packages, udev rule, build, smoke test
- [docs/usage.md](docs/usage.md) — launch arguments, consuming events, QoS, recording
- [docs/tuning.md](docs/tuning.md) — fps, thresholds/biases, ERC, noise filtering
- [docs/calibration.md](docs/calibration.md) — camera_info, rectification, TF frames
- [docs/multi_camera.md](docs/multi_camera.md) — running 2+ cameras, sync, per-camera calibration
- [docs/troubleshooting.md](docs/troubleshooting.md) — camera not found, permissions, no events

## License

Apache 2.0 (see [LICENSE](LICENSE)). This repo never contains proprietary
Metavision SDK files; the driver and OpenEB are installed separately via apt.

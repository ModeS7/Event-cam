# Event-cam — Plug-and-play ROS 2 setup for the Prophesee EVK4

ROS 2 launch files, configuration, and example code for getting a Prophesee
EVK4 event camera publishing events with minimal setup. This repo **wraps**
the community [metavision_driver](https://github.com/ros-event-camera/metavision_driver)
— it contains no driver code of its own, only launch/config glue,
example consumers, and documentation.

> **Status:** core path validated end-to-end on camera hardware
> (2026-06-05): install, build, live events, visualization, recording,
> playback, and both example consumers (Python + C++). The diagnostics
> watchdog (`evk4_diagnostics`) and the composed example launch
> (`event_rate_composed.launch.py`) are written and build-checked but not
> yet exercised on hardware.

## Supported platform

| Component | Version |
|---|---|
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| Camera | Prophesee EVK4 HD (Sony IMX636, EVT3) |
| Driver | `ros-jazzy-metavision-driver` 3.0.0 (apt) |

## Quickstart

```bash
# 1. Install the driver stack (bundles OpenEB — no separate Metavision SDK)
sudo apt install ros-jazzy-metavision-driver ros-jazzy-event-camera-renderer \
                 ros-jazzy-event-camera-py ros-jazzy-diagnostic-updater

# 2. Install the udev rule (one-time; see docs/installation.md §3), then
#    replug the camera
sudo wget -O /etc/udev/rules.d/88-cyusb.rules \
  https://raw.githubusercontent.com/prophesee-ai/openeb/main/hal_psee_plugins/resources/rules/88-cyusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# 3. Build this repo in a colcon workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/ModeS7/Event-cam.git
cd ~/ros2_ws && colcon build && source install/setup.bash

# 4. Launch and look at events (wave a hand — event cameras need motion!)
ros2 launch evk4_bringup evk4.launch.py
ros2 run rqt_image_view rqt_image_view /event_camera/image_raw
```

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

Raw events are the processing contract — decode them with
[event_camera_py](https://github.com/ros-event-camera/event_camera_py) (Python)
or [event_camera_codecs](https://github.com/ros-event-camera/event_camera_codecs) (C++).
The image topic exists for instant visualization (`rqt_image_view`); disable it
with `viz:=false`. **Note:** events are published best-effort — subscribers
need sensor-data QoS (see [docs/usage.md](docs/usage.md)).

## Repository layout

| Path | Purpose |
|---|---|
| `evk4_bringup/` | Launch file (`evk4.launch.py`), driver parameters, bias configs |
| `evk4_examples/` | Example Python subscriber (`ros2 run evk4_examples event_rate`) |
| `evk4_examples_cpp/` | Same example in C++, as a composable component |
| `evk4_diagnostics/` | Stream-health watchdog publishing on `/diagnostics` |
| `docs/` | Installation, usage, troubleshooting |

## Documentation

- [docs/installation.md](docs/installation.md) — apt packages, udev rule, build, smoke test
- [docs/usage.md](docs/usage.md) — launch arguments, consuming events, QoS, recording
- [docs/troubleshooting.md](docs/troubleshooting.md) — camera not found, permissions, no events

## License

Apache 2.0 (see [LICENSE](LICENSE)). This repo never contains proprietary
Metavision SDK files; the driver and OpenEB are installed separately via apt.

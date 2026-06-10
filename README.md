# Event-cam — Plug-and-play ROS 2 setup for the Prophesee EVK4

ROS 2 launch files, configuration, example code, and an **OpenEB-based driver**
for getting a Prophesee EVK4 event camera publishing events with minimal setup.

An **event camera** has no frames: each pixel independently reports the moments
its brightness changes ("events", microsecond-timed, ~millisecond latency).
That gives extreme dynamic range and no motion blur — and a data stream that
needs different tooling than normal video, which is what this repo provides.

The `evk4_driver` node is built directly on
[OpenEB](https://github.com/prophesee-ai/openeb) (the open edition of the
Metavision SDK) and exposes every on-sensor feature the EVK4 supports (rate
limiting, noise/flicker filters, region cropping, pixel masking, sync); the
rest of the repo is launch/config glue, example consumers, calibration, and
documentation.

> **Status:** `evk4_driver` (OpenEB-based) is validated end-to-end on a
> Raspberry Pi 5 (2026-06-09) — live events, renderer, Python/C++ examples,
> calibration, and recording — exposing all on-sensor facilities (ERC,
> Trail/STC, ROI, sync, AFK, Digital Crop, Event Mask). x86 re-validation of
> the new driver is pending.

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

Prerequisite: Ubuntu 24.04 with ROS 2 Jazzy installed (any platform — see
[docs/installation.md](docs/installation.md) for Humble, Raspberry Pi
specifics, and what each step does).

```bash
# 1. Clone into a colcon workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/ModeS7/Event-cam.git

# 2. Install dependencies + udev rule: OpenEB (Metavision SDK open edition)
#    via apt, event_camera_renderer from source. Then unplug/replug the camera.
export ROS_DISTRO=jazzy
~/ros2_ws/src/Event-cam/setup/install_deps.sh

# 3. Build and run (wave a hand — event cameras need motion!)
cd ~/ros2_ws
source ~/workspaces/3rd_party_ws/install/setup.bash
colcon build --symlink-install && source install/setup.bash
ros2 launch evk4_bringup evk4.launch.py
ros2 run rqt_image_view rqt_image_view /event_camera/image_raw
```

![Rendered event stream in rqt_image_view](docs/images/rqt_image_view.png)

*Live EVK4 event stream rendered to an image (camera panning across the
lab): blue = ON events (brightness increased), red = OFF events
(brightness decreased).*

Full instructions: [docs/installation.md](docs/installation.md).

The camera starts at **stock sensor defaults**. For the validated tuned setup
(noise suppression + an event-rate cap that keeps small boards responsive),
see the recipe at the top of [docs/tuning.md](docs/tuning.md).

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
| `evk4_driver/` | The camera driver: C++ composable node on OpenEB, publishes raw EVT3 events, exposes all sensor facilities |
| `evk4_bringup/` | Launch file, driver params, bias + calibration configs, `camera_info` helper |
| `evk4_examples/` | Example Python subscriber (`ros2 run evk4_examples event_rate`) |
| `evk4_examples_cpp/` | Same example in C++, as a composable component |
| `evk4_calibration/` | Guided intrinsic calibrator (`ros2 run evk4_calibration calibrate`) |
| `setup/` | `install_deps.sh` (one-command dependency setup) + vendored udev rule |
| `docs/` | Installation, usage, troubleshooting |

## Documentation

In reading order — each page builds on the previous one:

- [docs/installation.md](docs/installation.md) — apt packages, udev rule, build, smoke test
- [docs/usage.md](docs/usage.md) — launch arguments, consuming events, QoS, recording
- [docs/tuning.md](docs/tuning.md) — fps, thresholds/biases, ERC, noise filtering
- [docs/calibration.md](docs/calibration.md) — camera_info, rectification, TF frames
- [docs/multi_camera.md](docs/multi_camera.md) — running 2+ cameras, sync, per-camera calibration
- [docs/troubleshooting.md](docs/troubleshooting.md) — camera not found, permissions, no events

## License

Apache 2.0 (see [LICENSE](LICENSE)). This repo never contains proprietary
Metavision SDK files; OpenEB (the open edition) is installed separately via
apt, and our driver builds against it from this repo.

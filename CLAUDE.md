# CLAUDE.md — Event-cam project context

Public, open base repo giving plug-and-play ROS 2 access to a Prophesee EVK4
event camera. Goal: any student/researcher installs it, gets an event topic
publishing with minimal setup, and builds on top. Priorities: easy to install,
well-documented, easy to extend.

## Fixed decisions (do not revisit without a hard technical reason)

- **Platform:** Ubuntu 24.04 + ROS 2 Jazzy. Camera: Prophesee EVK4 (Sony
  IMX636, EVT3 event format).
- **Wrap, don't write, the driver:** use `ros-event-camera/metavision_driver`
  via apt. This repo contains only launch files, config, example consumer
  nodes, and docs.
- **License:** Apache 2.0. Never vendor or commit proprietary Metavision SDK
  files. Driver + OpenEB are installed by the user via apt.
- **Topic contract (user-confirmed):** default launch publishes BOTH
  `/event_camera/events` (`event_camera_msgs/EventPacket`, always) and
  `/event_camera/image_raw` (`sensor_msgs/Image`, renderer, behind
  `viz:=true`, default true).
- **Layout (user-confirmed):** `evk4_bringup` (launch + config + biases,
  ament_cmake), `evk4_examples` (ament_python, subscriber using
  `event_camera_py`), and `evk4_examples_cpp` (ament_cmake, same example
  as a composable component using `event_camera_codecs`; added 2026-06-05
  at user request — one package per language, like the ROS 2 demos).
- **Run model:** code is authored on a dev machine; runs on a separate lab PC
  with the camera. The lab PC only does anonymous `git pull` of this public
  repo and holds **no credentials**. Docs must never assume push access or
  secrets on the lab PC.

## Engineering principles

- Wrap, don't reinvent: reuse `metavision_driver`, `event_camera_codecs`,
  `event_camera_renderer`, `event_camera_py`, `event_camera_tools`.
- DRY, KISS, SRP. Clean separation: driver/launch config vs example consumer
  nodes vs docs.
- Fail fast and loud: clear errors if the camera isn't found or the driver
  isn't installed.
- Pin versions; document exact apt packages and the distro.
- No speculative features.

## Verified ecosystem facts (researched 2026-06, do not re-derive)

- **Install:** `sudo apt install ros-jazzy-metavision-driver` → v3.0.0
  (2025-12-06). Depends on `openeb_vendor`, so OpenEB lands under
  `/opt/ros/jazzy` — no separate Metavision SDK install. Apache 2.0.
- **Driver node** (verified against the released 3.0.0 source + live
  hardware):
  - Publishes `~/events` → `event_camera_msgs/msg/EventPacket`
    (binary EVT3 payload; best-effort QoS, KeepLast, default queue 1000).
  - Services (`std_srvs/srv/Trigger`): `~/save_biases`, `~/save_settings`.
  - Key params: `bias_file`, `serial`, `event_message_time_threshold`
    (default 1 ms), `erc_mode`/`erc_rate` (EVK4 event-rate control),
    `trail_filter*`, `roi`, `sync_mode`, `use_multithreading`.
  - Gen4 sensors (EVK4) do **not** support trigger-out.
  - Upstream launch files: `driver_node.launch.py`,
    `driver_composition.launch.py`, `start_recording.launch.py` (Jazzy+).
- **udev:** apt install does NOT activate udev rules. EVK4 enumerates as
  Cypress FX3, USB `04b4:00f5`. The covering rule is `88-cyusb.rules`
  (vendor `04b4`, MODE 666; its `RUN+=cy_renumerate.sh` clause produces a
  harmless udev warning) — `99-evkv2.rules` is vendor `03fd`, EVK2 only.
  Docs instruct downloading `88-cyusb.rules` from the OpenEB repo to
  `/etc/udev/rules.d/`, then
  `sudo udevadm control --reload-rules && sudo udevadm trigger`, replug.
- **Renderer:** `ros-jazzy-event-camera-renderer` v3.0.0. Subscribes
  `~/events` (EventPacket), publishes `~/image_raw` (`sensor_msgs/Image` via
  image_transport — lazy, near-zero cost without subscribers). Params:
  `fps` (default 25), `display_type` (`time_slice` | `sharp`).
- **Decoding:** `ros-jazzy-event-camera-py` v3.0.0 (Python `Decoder` →
  NumPy arrays); `event_camera_codecs` (C++); `event_camera_tools`
  (CLI: echo, perf).
- **event_camera_py API (verified in upstream README):**
  `decoder.decode(msg)` (ROS 2 EventPacket), then
  `decoder.get_cd_events()` → structured NumPy array, fields
  `x` (u2), `y` (u2), `p` (i1), `t` (i4, sensor time in µs);
  `get_ext_trig_events()` for trigger events.
- **event_camera_codecs C++ API (verified against the 3.0.0 headers):**
  subclass `event_camera_codecs::EventProcessor` — `void eventCD(...)`,
  `bool eventExtTrigger(...)` (returns continue/stop!), `void finished()`,
  `void rawData(...)`, all pure virtual. Then
  `DecoderFactory<EventPacket, Proc>::getInstance(*msg)` (nullptr on
  unknown encoding) and loop `decoder->decode(*msg, &proc)` until it
  returns false. `event_camera_codecs::EventPacket` aliases the ROS msg.
- **Renderer node (verified in upstream launch/source):** executable
  `renderer_node`, subscribes `~/events`, publishes `~/image_raw` — our
  launch remaps both into the camera namespace.
- **Composition (verified in upstream `driver_composition.launch.py`):**
  component plugins are `metavision_driver::DriverROS2` and
  `event_camera_renderer::Renderer`; upstream composes them in a
  `component_container_isolated` with `use_intra_process_comms: True`.
  Our `evk4.launch.py` does the same (decision 2026-06-05: avoid copying
  the event stream between driver and renderer). High-throughput user
  nodes should be C++ components loaded into the same container.
- **Statistics:** the driver prints bandwidth/rate stats to the launch
  terminal every second (`statistics_print_interval`). The `out` counter
  counts publish calls — made whenever ≥1 subscriber is matched,
  regardless of transport (verified in 3.0.0 source, line ~410). `out: 0`
  with `viz:=true` means the renderer is lazily unsubscribed (it only
  subscribes to events while something views `image_raw` — verified in
  renderer 3.0.0 source).
- **Released 3.0.0 vs master discrepancies (verified by grepping the
  3.0.0 tag of `driver_ros2.cpp` + live hardware 2026-06-05):** no
  `~/dump_statistics` service; no `frame_id` parameter (headers carry the
  last 4 serial digits, e.g. `1701`); `use_multithreading` default
  **true**; `trail_filter_threshold` default **0**;
  `event_message_size_threshold` default **1e9** (~off); trigger
  duty-cycle param is named `trigger_duty_cycle`. LESSON: upstream master
  README/source is ahead of the released package — verify against the
  3.0.0 tag (or the installed binary) before documenting parameters.

## Roadmap

Done:
- [x] Repo basics: LICENSE (Apache 2.0), .gitignore, README, CLAUDE.md;
      public remote https://github.com/ModeS7/Event-cam
- [x] `evk4_bringup`: `evk4.launch.py` (composed driver + optional
      renderer; `viz`, `bias_file`, `serial`, `camera_name` args; fails
      loud if a wrapped package is missing) + `evk4_params.yaml`
- [x] `evk4_examples`: `ros2 run evk4_examples event_rate` (sensor-data
      QoS, decodes with `event_camera_py`, logs Mev/s, ON%, msgs/s)
- [x] `evk4_examples_cpp`: same example as a C++ composable component.
      Both modes validated on hardware 2026-06-05: standalone
      (`ros2 run evk4_examples_cpp event_rate`, ~2-3 Mev/s live) and
      `ros2 component load` into the camera container (loads as
      `/event_rate_cpp`, stats appear in the launch terminal).
- [x] `docs/`: installation, usage (incl. recording/playback),
      troubleshooting
- [x] Full hardware validation on the lab PC (2026-06-05): install, udev,
      build, launch (viz on/off), `rqt_image_view`, `event_rate`
      (~1.5–7 Mev/s live), bag record + playback, clean shutdown.
      Findings folded into docs/launch/params (see verified-facts).

Next: nothing planned — extend as research needs arise.

## Conventions

- No per-file license headers (user decision 2026-06-05): the root LICENSE
  file covers the repo. Do not add copyright boilerplate to source files.
- Verify package XML/builds with `colcon build` where Jazzy is available;
  otherwise validate `package.xml` as XML and note the limitation.
- Reference upstream docs rather than duplicating them; pin exact package
  versions in docs.

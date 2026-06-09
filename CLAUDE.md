# CLAUDE.md — Event-cam project context

Public, open base repo giving plug-and-play ROS 2 access to a Prophesee EVK4
event camera. Goal: any student/researcher installs it, gets an event topic
publishing with minimal setup, and builds on top. Priorities: easy to install,
well-documented, easy to extend.

## Fixed decisions (do not revisit without a hard technical reason)

- **Platform (tiered, decision 2026-06-08):** the install path is keyed on
  CPU **architecture**, not the specific board — x86 gets OpenEB from apt,
  ARM must source-build it; distro just sets `$ROS_DISTRO`. Tier 1 (validated)
  = x86_64 + Ubuntu 24.04 + Jazzy. Tier 2 (expected, untested) = x86_64 +
  22.04 + Humble. Tier 3 (experimental) = any ARM64 SBC on Jazzy or Humble.
  **Chosen deployment board (2026-06-08): Raspberry Pi 5 16GB**, to run
  Ubuntu 24.04 + Jazzy — i.e. ARM64 but the SAME distro as the validated x86
  target, so the only real unknown is building OpenEB on ARM. Pi setup (flash
  OS, build OpenEB, clone, colcon) happens on the Pi itself; not yet done. A
  future RGB CSI camera on the Pi uses the libcamera/V4L2 path (not Jetson's
  Argus). Ubuntu < 22.04 unsupported. Camera: Prophesee EVK4 (IMX636, EVT3).
  Docs use one `installation.md`: a tier table carries the
  validated/expected/experimental status, only step 2 branches by platform,
  steps 3–6 are shared. Keep it scannable — one audience line at the top,
  NOT per-section role badges (tried 2026-06-08, too noisy, removed).
- **Wrap, don't write, the driver:** use `ros-event-camera/metavision_driver`
  via apt. This repo contains only launch files, config, example consumer
  nodes, and docs.
- **License:** Apache 2.0. Never vendor or commit proprietary Metavision SDK
  files. Driver + OpenEB are installed by the user separately (apt on x86;
  apt or source build on ARM). The vendored `88-cyusb.rules` is the only
  third-party file we ship, and it is Apache-2.0 from OpenEB.
- **Topic contract (user-confirmed):** default launch publishes BOTH
  `/event_camera/events` (`event_camera_msgs/EventPacket`, always) and
  `/event_camera/image_raw` (`sensor_msgs/Image`, renderer, behind
  `viz:=true`, default true).
- **Layout (user-confirmed):** `evk4_bringup` (launch + config + biases +
  calibration + camera_info helper, ament_cmake), `evk4_examples`
  (ament_python, subscriber using `event_camera_py`), `evk4_examples_cpp`
  (ament_cmake, same example as a composable component using
  `event_camera_codecs`; one package per language, like the ROS 2 demos),
  and `evk4_calibration` (ament_python, guided OpenCV intrinsic calibrator).
- **Diagnostics (decision 2026-06-08):** a surface-level `evk4_diagnostics`
  watchdog (subscribe to `/events`, report OK/WARN/ERROR rates) was built
  then **removed** — the real need is *driver-level* diagnostics (USB
  errors, sensor status, internal queue depth) which only the driver can
  expose. Do NOT re-add a downstream rate-watcher. If pursued, the path is
  upstream (feature-request/contribute to `metavision_driver`) or parsing
  the driver's existing stats log — not a new subscriber node.
- **Exposed driver params (policy, 2026-06-09):** broadly-useful
  mode-selectors are launch args even absent a specific use case (lesson
  from sync_mode). Args now include `sync_mode`, `trigger_in_mode` (external
  trigger input — IMU/RGB/sensor sync), `settings` (pixel-mask JSON; also
  the `save_settings` target), and `params_file` (escape hatch overriding
  the whole driver YAML → any driver param without enumerating). Long tail
  (`roni`, `mipi_frame_period`, `statistics_print_interval`,
  `send_queue_size`, `event_message_size_threshold`) is documented-commented
  in evk4_params.yaml. NOT exposed: `trigger_out_*` (EVK4/Gen4 unsupported),
  `from_file` (rosbag play covers it).
- **Multi-camera readiness (decision 2026-06-09):** the bringup is
  parameterized per camera (`camera_name`+`serial`), so N cameras = launch
  N times into separate namespaces/containers; biases and `calibration_url`
  are already per-launch. Door-openers added: renderer is namespaced under
  `camera_name` (was `/renderer` — would clash across cameras), and
  `sync_mode` (standalone/primary/secondary) is a launch arg for hardware
  time-sync. **Extrinsic/stereo calibration is deliberately NOT built** —
  untestable with one camera; documented in multi_camera.md as the future
  extension (a future `evk4_calibration` stereo mode), building on the
  per-camera intrinsics/frames/sync that already exist. Do not guess at it
  without real multi-camera hardware.
- **Run model:** code is authored on a dev machine; runs on a separate
  runtime machine with the camera (lab PC now, Raspberry Pi 5 next).
  That machine only does anonymous `git pull` of this public repo and holds
  **no credentials**. Docs must never assume push access or secrets there.

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
  It is **vendored** at `setup/udev_rules/88-cyusb.rules` (Apache-2.0 from
  OpenEB — allowed, not proprietary SDK) and copied to `/etc/udev/rules.d/`
  by the setup script / docs, then `udevadm control --reload-rules &&
  udevadm trigger`, replug. (Deliberately NOT vendoring pandect-setup's
  `99-usb.rules`, which is MODE 0666 on ALL usb — a security downgrade.)
- **Setup script + workspaces (decision 2026-06-08, modeled on
  AIS-CPS-Lab/pandect-setup):** `setup/install_deps.sh` does a hybrid
  install — apt for `metavision_driver` + `event_camera_py` ONLY, and
  **always source-build** `event_camera_renderer` (+ `event_camera_msgs`/
  `_codecs` via `vcs import` of the renderer's `.repos`) into
  `~/workspaces/3rd_party_ws` with `--symlink-install --cmake-args
  -DCMAKE_BUILD_TYPE=RelWithDebInfo` (RelWithDebInfo matters: high event rate
  makes Debug slow). Installs the vendored udev rule, and appends deps-ws
  sourcing to `~/.bashrc` AND `/etc/skel/.bashrc`. No emoji, `set -euo
  pipefail`, re-runnable. **CORRECTION 2026-06-09:** I had briefly changed
  this to apt-prefer the renderer (source only as fallback), but that
  defeated the teacher's (AIS-CPS-Lab) EXPLICIT design — the renderer +
  decode libs must ALWAYS be a separate SOURCE workspace (modifiable,
  separated from apt and from our overlay). Reverted to always-source on
  every platform. Do NOT re-introduce the apt-prefer shortcut.
  **arm64 confirmed (2026-06-09): the apt parts
  (`ros-jazzy-metavision-driver`/`-py`/`-codecs`/`-renderer`) all have
  arm64 binaries (3.0.0, 2026-04-12) — so OpenEB needs NO source build; only
  the renderer is built from source, by design.** **FULL hardware validation
  on the Pi 5 (2026-06-09):**
  apt install → colcon build → driver opens camera (IMX636, serial 00051701,
  1280x720) → events ~245 msgs/s → renderer image_raw ~23.5 Hz → composed
  pipeline only ~7% CPU on the Pi 5. Tier 3 ARM is effectively Tier 1 on
  this board. (Pi accessed via Tailscale SSH as user `mode`, sudo pw shared
  by user; gnome-shell ~44% CPU is the remote-desktop software-render issue,
  not the camera — go headless for continuous use.)
- **Renderer:** `ros-jazzy-event-camera-renderer` v3.0.0. Subscribes
  `~/events` (EventPacket), publishes `~/image_raw` (`sensor_msgs/Image` via
  image_transport — lazy, near-zero cost without subscribers). Params:
  `fps` (default 25), `display_type` (`time_slice` | `sharp`) — both exposed
  as `evk4.launch.py` args. Copies the event packet header onto image_raw
  (verified renderer.cpp:152), so image_raw frame_id == driver frame_id
  (serial tail on 3.0.0). Publishes NO camera_info.
- **Tuning + calibration (decision 2026-06-09, not yet hw-validated):**
  `evk4.launch.py` gained `fps`/`display_type` (→ renderer), `frame_id`
  (→ driver; 3.0.0 ignores it, uses serial tail), and `calibration_url`
  (→ starts `camera_info_publisher.py`). camera_info has no native source,
  so `evk4_bringup/scripts/camera_info_publisher.py` (Python node installed
  from the ament_cmake pkg via install(PROGRAMS)) loads a standard
  camera_info YAML and republishes CameraInfo copying each image_raw header
  (stamp+frame_id) → image_proc rectify pairs them cleanly. Calibration
  YAMLs live in `config/calibration/` (committed one is a zero-distortion
  PLACEHOLDER). Docs: tuning.md (fps/biases/ERC), calibration.md
  (capture→rectify→TF). Biases are runtime params (rqt_reconfigure).
- **Calibration tool (decision 2026-06-09, user-confirmed):** built our OWN
  guided calibrator `evk4_calibration` (ament_python, deps rclpy/sensor_msgs/
  cv_bridge/opencv/numpy) — `ros2 run evk4_calibration calibrate`. OpenCV
  window: live checkerboard detect + cornerSubPix, X/Y/Size/Skew coverage
  bars, AUTO-captures distinct views (SPACE forces, `c` calibrates), runs
  `cv2.calibrateCamera`, writes a camera_info YAML. **Deliberately DROPPED
  E2VID/e2calib** (user decision): deep-learning reconstruction + Kalibr are
  too heavy / not "just works", esp. on Pi — do NOT add them back. Goal:
  students run one command, follow on-screen coverage, done. NOT
  hw-validated; the real unknown is whether the event-rendered checkerboard
  detects reliably (flickering board on a screen is the recommended capture).
  Needs a display (X-forward/VNC on headless Pi).
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
      `event_rate_composed.launch.py` (camera + component, one command)
      added later — NOT yet run on the lab PC.
- [x] `docs/`: installation (tiered, arch-keyed), usage (incl.
      recording/playback), troubleshooting, tuning, calibration
- [x] Tuning + calibration + frame_id (2026-06-09, NOT hw-validated):
      fps/display_type/frame_id/calibration_url launch args,
      camera_info_publisher.py, config/calibration/ template,
      docs/tuning.md + docs/calibration.md
- [x] `evk4_calibration` (2026-06-09, NOT hw-validated): guided OpenCV
      intrinsic calibrator (auto-capture + coverage bars), writes
      camera_info YAML. E2VID/e2calib deliberately not used.
- [x] Full hardware validation on the lab PC (2026-06-05): install, udev,
      build, launch (viz on/off), `rqt_image_view`, `event_rate`
      (~1.5–7 Mev/s live), bag record + playback, clean shutdown.
      Findings folded into docs/launch/params (see verified-facts).

Next: nothing planned — extend as research needs arise.

## Conventions

- **Workspaces (decision 2026-06-08, revised 2026-06-09):** ALWAYS 3 layers
  on every platform (matches the teacher's AIS-CPS-Lab convention):
  `/opt/ros` (ROS + apt driver/decoder) → `~/workspaces/3rd_party_ws`
  (source-built `event_camera_renderer` + `event_camera_msgs`/`_codecs`) →
  `~/ros2_ws` (this repo). The middle layer is source ON PURPOSE so the
  rendering/decoding code is modifiable and separated — not an apt fallback.
  Build deps once so editing our packages only rebuilds the small overlay.
  Workspace name `~/workspaces/3rd_party_ws` must match the setup script and
  docs. Documented in installation.md (step 3 + Workspaces section).
- **Portability (keep it this way):** never hardcode a ROS distro in code.
  `package.xml`/`CMakeLists.txt` reference ROS package names (distro/arch
  agnostic; `rosdep` resolves per platform); `rosdep install --from-paths
  src` is the canonical install step; the launch `_require()` builds its
  apt hint from `$ROS_DISTRO`. The only irreducibly non-portable piece is
  OpenEB-on-ARM (no upstream binary → documented source build). Future
  features must preserve this — no `ros-jazzy-*` literals in code.
- No per-file license headers (user decision 2026-06-05): the root LICENSE
  file covers the repo. Do not add copyright boilerplate to source files.
- Verify package XML/builds with `colcon build` where Jazzy is available;
  otherwise validate `package.xml` as XML and note the limitation.
- Reference upstream docs rather than duplicating them; pin exact package
  versions in docs.

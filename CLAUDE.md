# CLAUDE.md — Event-cam project context

Public, open base repo giving plug-and-play ROS 2 access to a Prophesee EVK4
event camera. Goal: any student/researcher installs it, gets an event topic
publishing with minimal setup, and builds on top. Priorities: easy to install,
well-documented, easy to extend.

## Fixed decisions (do not revisit without a hard technical reason)

- **Our own driver on OpenEB (decision 2026-06-09 — REVERSES the earlier
  "wrap metavision_driver"):** `evk4_driver` is our C++ composable component
  built directly on **OpenEB** (the open edition of the Metavision SDK),
  exposing **every on-sensor facility the EVK4/IMX636 supports** — which the
  community `metavision_driver` did not (it wrapped only a subset: no AFK,
  Digital Crop, or Event Mask). The community driver is no longer used. We
  still reuse the `ros-event-camera` ecosystem around the driver:
  `event_camera_msgs` (the raw-EVT3 `EventPacket` wire format),
  `event_camera_renderer`, `event_camera_codecs`, `event_camera_py`,
  `event_camera_tools`. Full rationale + the facility-coverage audit + the
  ARM/licensing analysis: memory `sdk-rewrite-decision`. The full proprietary
  Metavision SDK (CV/ML/Calibration/Analytics) is x86-only + license-gated, so
  it is NOT a core dependency — it is a future OPT-IN layer
  (`evk4_sdk_advanced`) layered on top of our open `EventPacket` stream.
- **Platform (decision 2026-06-08, updated 2026-06-09):** the install is the
  SAME on x86_64 and ARM64 — OpenEB ships both arm64 and x86 apt binaries
  (`openeb_vendor`), so there is no per-architecture branch. **Chosen
  deployment board: Raspberry Pi 5 16GB** running Ubuntu 24.04 + Jazzy, FULLY
  validated 2026-06-09 (driver opens the camera, streams, renders, ~7% CPU). A
  future RGB CSI camera on the Pi uses the libcamera/V4L2 path (not Jetson's
  Argus). Ubuntu < 22.04 unsupported. Camera: Prophesee EVK4 (IMX636, EVT3).
  Docs use one `installation.md` with a platform table for validated/expected
  status; the install steps are uniform across platforms.
- **License:** Apache 2.0. Never vendor or commit proprietary Metavision SDK
  files. **OpenEB** is installed by the user via apt
  (`ros-$ROS_DISTRO-openeb-vendor`); our driver builds from this repo against
  it. The vendored `88-cyusb.rules` is the only third-party file we ship, and
  it is Apache-2.0 from OpenEB.
- **Topic contract (user-confirmed):** default launch publishes
  `/event_camera/events` (`event_camera_msgs/EventPacket`, raw EVT3, always),
  `/event_camera/image_raw` (`sensor_msgs/Image`, renderer, behind
  `viz:=true`, default true), and `/event_camera/camera_info`
  (`sensor_msgs/CameraInfo`, when `calibration_url` is set).
- **Layout (user-confirmed):** `evk4_driver` (the OpenEB-based driver:
  ament_cmake C++ composable component `evk4_driver::EVK4Driver` + standalone
  `driver_node`), `evk4_bringup` (launch + config + biases + calibration +
  camera_info helper, ament_cmake), `evk4_examples` (ament_python, subscriber
  using `event_camera_py`), `evk4_examples_cpp` (ament_cmake, composable
  component using `event_camera_codecs`; one package per language, like the
  ROS 2 demos), and `evk4_calibration` (ament_python, guided OpenCV intrinsic
  calibrator). Future: `evk4_sdk_advanced` (opt-in, full Metavision SDK Pro).
- **Diagnostics (decision 2026-06-08, updated 2026-06-09):** a surface-level
  `evk4_diagnostics` downstream rate-watcher was built then **removed** — do
  NOT re-add it. Real driver-level diagnostics (USB errors, sensor status,
  internal queue depth) now belong IN `evk4_driver` — we own the driver and
  the SDK exposes them, so there is nothing to upstream. Not yet built.
- **Exposed facilities/params (policy, 2026-06-09):** `evk4_driver` exposes
  every EVK4 facility as ROS params (set in `evk4_params.yaml`, applied at
  camera start): ERC (`erc_mode`/`erc_rate`), Trail/STC (`trail_filter*`),
  ROI/RONI (`roi`/`roni`), AFK (`afk_*`), Digital Crop (`digital_crop_*`),
  Event Mask (`event_mask_pixels`). `eraf_*` params exist but the IMX636 does
  NOT implement the Event Rate Activity Filter (skipped with a warning).
  Biases are tuned at RUNTIME (`ros2 param set`). Mode-selectors are launch
  args: `serial`, `bias_file`, `frame_id`, `sync_mode`, `trigger_in_mode`,
  `settings` (camera-settings JSON; also the `save_settings` target),
  `calibration_url`, `viz`, `fps`, `display_type`, plus `params_file` (escape
  hatch swapping the whole driver YAML). NOT exposed: trigger-OUT (EVK4/Gen4
  unsupported).
- **Multi-camera readiness (decision 2026-06-09):** the bringup is
  parameterized per camera (`camera_name`+`serial`), so N cameras = launch
  N times into separate namespaces/containers; biases and `calibration_url`
  are already per-launch. Door-openers: renderer is namespaced under
  `camera_name` (was `/renderer` — would clash across cameras), and
  `sync_mode` (standalone/primary/secondary) is a launch arg for hardware
  time-sync. **Extrinsic/stereo calibration is deliberately NOT built** —
  untestable with one camera; documented in multi_camera.md as the future
  extension (a future `evk4_calibration` stereo mode), building on the
  per-camera intrinsics/frames/sync that already exist. Do not guess at it
  without real multi-camera hardware.
- **Run model:** code is authored on a dev machine; runs on a separate
  runtime machine with the camera (lab PC and Raspberry Pi 5). That machine
  only does anonymous `git pull` of this public repo and holds **no
  credentials**. Docs must never assume push access or secrets there. (The
  dev machine pushes; it is set up with an SSH key to github.com.)

## Engineering principles

- Build on, don't reinvent: our driver wraps the OpenEB SDK (`Metavision::`
  HAL/Camera), not a from-scratch USB/decode stack; reuse
  `event_camera_codecs`, `event_camera_renderer`, `event_camera_py`,
  `event_camera_tools` and the `EventPacket` wire format.
- DRY, KISS, SRP. Clean separation: driver vs launch/config vs example
  consumer nodes vs docs.
- Fail fast and loud: clear errors if the camera isn't found or a dependency
  is missing.
- Pin versions; document exact apt packages and the distro.
- No speculative features.

## Verified ecosystem facts (researched 2026-06, do not re-derive)

- **Install:** `sudo apt install ros-$ROS_DISTRO-openeb-vendor` (OpenEB =
  Metavision SDK open edition; `openeb_vendor` v2.0.2, SDK **5.0.0**; arm64 +
  x86 binaries) + `ros-$ROS_DISTRO-event-camera-py`. Installs under
  `/opt/ros/$ROS_DISTRO/opt/openeb_vendor/`. No separate/proprietary SDK.
- **evk4_driver (our driver — verified + hardware-validated on the Pi 5,
  2026-06-09):**
  - Opens the EVK4 via `Metavision::Camera::from_serial` /
    `from_first_available` (with `DeviceConfig.set_format("EVT3")`); raw EVT3
    arrives via `cam_.raw_data().add_callback(...)` and is `memcpy`'d straight
    into an `EventPacket` (`encoding="evt3"`, header stamp = system-clock ns),
    published on `~/events` with QoS `KeepLast(1000).best_effort()
    .durability_volatile()` — matches the old driver so the renderer/codecs
    subscribe unchanged. Publishes lazily (only while subscribed).
  - Facilities via `cam_.get_device().get_facility<Metavision::I_*>()`:
    `I_LL_Biases`, `I_ErcModule`, `I_EventTrailFilterModule`, `I_ROI`,
    `I_TriggerIn`, `I_CameraSynchronization`, `I_AntiFlickerModule`,
    `I_DigitalCrop`, `I_DigitalEventMask`. Headers
    `<metavision/hal/facilities/i_*.h>`. `I_EventRateActivityFilterModule`
    returns null on the IMX636 (skipped). Biases: `get_all_biases()` enumerate
    (skip the computed `bias_diff`), declare each as a live int param applied
    on change via `set()`.
  - Services (`std_srvs/srv/Trigger`): `~/save_biases`, `~/save_settings`
    (`I_LL_Biases::save_to_file` / `cam_.save`); both fail clearly if no
    `bias_file`/`settings` path was given at startup.
  - Honors `frame_id` (default `event_camera_optical_frame`).
  - **Tracked gaps:** single-threaded raw callback (no `use_multithreading`
    yet; fine at tested rates, ~7.9% CPU) and NO per-second stats line.
  - CMake: probe `find_package(MetavisionSDK COMPONENTS driver QUIET)` for the
    version, then require components `driver` (<5) or `base core stream`
    (>=5); `add_definitions(-DMETAVISION_VERSION=<major>)`; the Camera header
    is `metavision/sdk/driver/camera.h` (<5) vs `.../stream/camera.h` (>=5).
    `ldd` confirms the component links only `openeb_vendor` libs.
- **udev:** apt install does NOT activate udev rules. EVK4 enumerates as
  Cypress FX3, USB `04b4:00f5`. The covering rule is `88-cyusb.rules`
  (vendor `04b4`, MODE 666; its `RUN+=cy_renumerate.sh` clause produces a
  harmless udev warning) — `99-evkv2.rules` is vendor `03fd`, EVK2 only.
  It is **vendored** at `setup/udev_rules/88-cyusb.rules` (Apache-2.0 from
  OpenEB) and copied to `/etc/udev/rules.d/` by the setup script / docs, then
  `udevadm control --reload-rules && udevadm trigger`, replug. (Deliberately
  NOT vendoring pandect-setup's `99-usb.rules`, MODE 0666 on ALL usb — a
  security downgrade.)
- **Setup script + workspaces (decision 2026-06-08, modeled on
  AIS-CPS-Lab/pandect-setup):** `setup/install_deps.sh` — apt for
  `openeb_vendor` + `event_camera_py` ONLY, and **always source-build**
  `event_camera_renderer` (+ `event_camera_msgs`/`_codecs` via `vcs import` of
  the renderer's `.repos`) into `~/workspaces/3rd_party_ws`
  (`--symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo` —
  RelWithDebInfo matters: high event rate makes Debug slow). Installs the
  vendored udev rule; appends deps-ws sourcing to `~/.bashrc` AND
  `/etc/skel/.bashrc`. No emoji, `set -euo pipefail`, re-runnable. The
  always-source-renderer separation is the teacher's (AIS-CPS-Lab) EXPLICIT
  design (renderer + decode libs are a modifiable SOURCE workspace, separate
  from apt and from our overlay) — do NOT re-introduce an apt-prefer shortcut.
  **Pi 5 validation (2026-06-09, our driver):** events ~248 msgs/s, renderer
  image_raw ~24 Hz, composed pipeline ~7.9% CPU. (On a Pi with a desktop
  session, go headless for continuous use — gnome-shell software rendering
  eats CPU, not the camera.)
- **Renderer:** `event_camera_renderer` (source-built into 3rd_party_ws).
  Subscribes `~/events` (EventPacket), publishes `~/image_raw`
  (`sensor_msgs/Image` via image_transport — lazy, near-zero cost without
  subscribers). Params `fps` (default 25), `display_type`
  (`time_slice`|`sharp`) — both exposed as `evk4.launch.py` args. Copies the
  EventPacket header onto image_raw, so `image_raw` frame_id == our driver's
  `frame_id` (now `event_camera_optical_frame`, honored). Publishes NO
  camera_info.
- **camera_info / calibration:** `evk4.launch.py` args `fps`/`display_type`
  (→ renderer), `frame_id` (→ driver; honored), `calibration_url` (→ starts
  the camera_info component). camera_info has no native source, so
  `evk4_bringup::CameraInfoPublisher` (C++ composable component, composed
  into the camera container with intra-process comms — a Python version
  cost ~50% of a Pi core touching every frame) loads a standard camera_info
  YAML via `camera_calibration_parsers` and republishes CameraInfo copying
  each image_raw header (stamp+frame_id) → `image_proc` rectify pairs them
  cleanly. Calibration YAMLs live in `config/calibration/`
  (committed one is a zero-distortion PLACEHOLDER). Docs: tuning.md, calibration.md.
- **Calibration tool (decision 2026-06-09; pattern switched 2026-06-10):**
  our OWN guided calibrator `evk4_calibration` (ament_python, deps rclpy/
  sensor_msgs/cv_bridge/opencv/numpy) — `ros2 run evk4_calibration calibrate`.
  Uses a **blinking ASYMMETRIC CIRCLE GRID** (`docs/circle_grid.html`,
  default `grid_size:=4x11`): detection = |B-R| polarity-contrast image →
  GaussianBlur → SimpleBlobDetector → `cv2.findCirclesGrid(ASYMMETRIC|
  CLUSTERING)`, on a worker thread (GUI never blocks); X/Y/Size/Skew coverage
  bars, auto-capture, `cv2.calibrateCamera` with `CALIB_FIX_K3` (narrow-FOV
  overfit guard), RMS written into the YAML header. **Checkerboards were
  tried and ABANDONED** (hardware experience 2026-06-10): saddle-point
  corners drift on speckle/gappy event edges even with blur — circle
  centroids average hundreds of events and are robust (same conclusion as
  E-Calib and eKalibr; cited in calibration.md). **Deliberately DROPPED
  E2VID/e2calib** (user decision): deep-learning reconstruction + Kalibr are
  too heavy / not "just works", esp. on Pi — do NOT add them back. Circle
  mode not yet hw-validated. Needs a display (X-forward/VNC on headless Pi).
  Note: Metavision's own Calibration module is x86-only + Pro-licensed, so
  our open OpenCV tool is the better fit for the base repo.
- **Decoding:** `ros-jazzy-event-camera-py` (Python `Decoder` → NumPy
  arrays); `event_camera_codecs` (C++); `event_camera_tools` (CLI: echo,
  perf). All consume our `EventPacket` unchanged.
- **event_camera_py API (verified in upstream README):**
  `decoder.decode(msg)` (ROS 2 EventPacket), then `decoder.get_cd_events()`
  → structured NumPy array, fields `x` (u2), `y` (u2), `p` (i1), `t` (i4,
  sensor time in µs); `get_ext_trig_events()` for trigger events.
- **event_camera_codecs C++ API (verified against the headers):** subclass
  `event_camera_codecs::EventProcessor` — `void eventCD(...)`,
  `bool eventExtTrigger(...)` (returns continue/stop!), `void finished()`,
  `void rawData(...)`, all pure virtual. Then
  `DecoderFactory<EventPacket, Proc>::getInstance(*msg)` (nullptr on unknown
  encoding) and loop `decoder->decode(*msg, &proc)` until it returns false.
  `event_camera_codecs::EventPacket` aliases the ROS msg.
- **Composition (decision 2026-06-05):** component plugins
  `evk4_driver::EVK4Driver` + `event_camera_renderer::Renderer` compose in a
  `component_container_isolated` with `use_intra_process_comms: True` — the
  raw EVT3 stream passes driver→renderer as a moved `unique_ptr` (no DDS
  serialization). Out-of-process subscribers (rosbag/rqt/your nodes) still get
  normal DDS copies. High-throughput user nodes should be C++ components
  loaded into the SAME container.

## Roadmap

Done:
- [x] Repo basics: LICENSE (Apache 2.0), .gitignore, README, CLAUDE.md;
      public remote https://github.com/ModeS7/Event-cam
- [x] Wrapper-era packages (evk4_bringup, evk4_examples, evk4_examples_cpp,
      evk4_calibration, docs), validated on the lab PC (x86, 2026-06-05) and
      the Pi 5 (2026-06-09). `event_rate` examples ~1.5–7 Mev/s live; bag
      record/playback; clean shutdown.
- [x] **SDK rewrite (2026-06-09):** replaced `metavision_driver` with our
      OpenEB-based `evk4_driver` — raw EVT3 `EventPacket` + every EVK4 facility
      (incl. AFK, Digital Crop, Event Mask the old driver could not reach),
      live biases, save_biases/save_settings, honored frame_id. It is the
      production driver in `evk4.launch.py`; whole pipeline (renderer,
      examples, calibration camera_info, bag) re-validated on the Pi. Install
      switched to `openeb_vendor`; docs truthed up. (Phases 0–3; details in
      memory `sdk-rewrite-decision`.)

Next:
- [ ] `evk4_sdk_advanced` — opt-in full Metavision SDK Pro layer
      (calibration/CV/ML/analytics) over our `EventPacket` stream. Blocked on
      the user's SDK Pro license + ARM source build.
- [ ] Robustness: optional multithreaded capture + per-second stats line.
- [ ] x86 re-validation of `evk4_driver` on the lab PC; high event-rate drop
      check (needs a dynamic scene — wave a hand with `event_rate` running).

## Conventions

- **Workspaces (decision 2026-06-08, revised 2026-06-09):** ALWAYS 3 layers
  on every platform (matches the teacher's AIS-CPS-Lab convention):
  `/opt/ros` (ROS + apt OpenEB/decoder) → `~/workspaces/3rd_party_ws`
  (source-built `event_camera_renderer` + `event_camera_msgs`/`_codecs`) →
  `~/ros2_ws` (this repo). The middle layer is source ON PURPOSE so the
  rendering/decoding code is modifiable and separated — not an apt fallback.
  Build deps once so editing our packages only rebuilds the small overlay.
  Workspace name `~/workspaces/3rd_party_ws` must match the setup script and
  docs.
- **Portability (keep it this way):** never hardcode a ROS distro in code.
  `package.xml`/`CMakeLists.txt` reference ROS package names (distro/arch
  agnostic; `rosdep` resolves per platform); the launch `_require()` hints
  "build this repo" for our `evk4_*` packages and a `$ROS_DISTRO`-derived apt
  name for external ones. OpenEB ships arm64 + x86 apt binaries, so there is
  no irreducibly non-portable piece. No `ros-jazzy-*` literals in code.
- No per-file license headers (user decision 2026-06-05): the root LICENSE
  file covers the repo. Do not add copyright boilerplate to source files.
- Verify package XML/builds with `colcon build` where Jazzy is available;
  otherwise validate `package.xml` as XML and note the limitation.
- Reference upstream docs rather than duplicating them; pin exact package
  versions in docs.

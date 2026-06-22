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
  it. Third-party files we ship: the vendored `88-cyusb.rules` (Apache-2.0,
  from OpenEB) and `setup/patches/event_camera_renderer-backlog-cap.patch`
  (a diff of Apache-2.0 ros-event-camera code).
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
  args: `serial`, `frame_id`, `sync_mode`, `trigger_in_mode`,
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

## Project protocol (all lessons consolidated 2026-06-11)

### Architecture
- Build on, don't reinvent: OpenEB SDK + the ros-event-camera ecosystem;
  own a component only when coverage demands it (the driver rewrite was
  justified by sensor-facility coverage, nothing less).
- **One mechanism per job**: one viewer for every image topic
  (`rqt_image_view`; nodes publish, viewers view — no bespoke GUI windows),
  one canonical launch (`evk4.launch.py`), one params hierarchy (stock yaml
  -> recommended yaml -> the user's `~/my_params.yaml`, which carries the
  WHOLE sensor setup including startup biases). One docs home per topic,
  cross-linked. Known deliberate duplication: the topic table lives in
  README AND usage.md — update both together.
- Defaults are stock and predictable; the tuned setup is explicit, shipped
  (`evk4_params_recommended.yaml`), and taught as a recipe whose output is
  the user's own artifact.
- High-rate paths are composable components with intra-process comms in ONE
  container; everything is lazy (publish/compute only while subscribed); no
  Python on per-frame hot paths (a Python header-copier cost half a Pi
  core; the same logic as a composed C++ component costs ~nothing).
- Processing loops need a cheap fast-path and a throttle (search only new
  frames, bounded OpenCV threads, clutter guards) or they starve the
  pipeline they serve.
- Live-view consumers take DEPTH-1 queues (newest frame or none): a deeper
  subscription queue fills once when the callback is briefly slow and then
  holds a standing several-hundred-ms backlog that never drains — the
  renderer slot-backlog lesson, one level up (calibrator overlay,
  2026-06-11: depth 10 = +0.36 s; depth 1 = tens of ms — full-res overlay
  ships at ~+34 ms vs image_raw, user-accepted).
- The source workspace (`3rd_party_ws`) is modifiable by design, but local
  patches are a liability — every variant of the renderer patch bit us.
  Any load-bearing local patch must be distributed or dropped. The one
  load-bearing patch — the renderer pending-frame cap (drop-NEWEST only:
  stock replays a stale backlog = seconds of lag after quiet; drop-oldest
  starves slow decoders into total silence) — is DISTRIBUTED as
  `setup/patches/event_camera_renderer-backlog-cap.patch`, applied by
  install_deps.sh (idempotent). Upstream PR still worth filing.

### Performance model (hardware-validated)
- Pipeline cost scales with EVENT RATE. ERC is the latency/CPU lever:
  10 Mev/s (~35 MB/s) renders smoothly on a Pi 5; a 100 M cap (~180 MB/s
  peaks) stutters unusably. Biases affect image quality and CPU only — in
  all testing they never measurably changed latency. Do not conflate them.
- `display_type` sharp = event-count frames: crisp on busy scenes,
  seconds-late on quiet ones (the count takes seconds to fill). Default
  `time_slice` is honest about time at any rate. Sharp is for deliberately
  busy scenes only.
- `fps` scales display-path cost roughly linearly; 25 is the Pi default.

### Validation
- Nothing is "done" until hardware-validated; written != proven (lens
  focus, corner drift, transport poisoning were all caught only on real
  hardware by a real user).
- Final validation is the STUDENT EXPERIENCE: the user runs the documented
  flow cold, exactly as written. It catches what headless probes cannot.
  Remote debugging is fine but transparent — announce every process
  started/stopped on the user's machines, never leave detached phantoms.
- Change one variable at a time; controlled A/B comparisons settle theories
  that days of reasoning cannot. When "it worked yesterday", suspect the
  code that changed since yesterday before inventing theories.
- Probes lie: `ros2 topic hz | grep` buffers (write to a file, read after
  exit); `find` skips directory symlinks; check what the running process
  actually has loaded (`/proc/PID/maps`) before trusting a rebuild.
- "Feels laggy" is LATENCY, not rate — `ros2 topic hz` is blind to it.
  Measure `now - header.stamp` at a subscriber (2026-06-11: the calibrator
  overlay published at full rate while running 0.4 s behind; three rate
  measurements missed what one latency probe settled).
- Keep achieved-performance numbers OUT of the student docs (decision
  2026-06-22, reverses the earlier "record validated numbers" rule): latency,
  achieved fps, CPU %, throughput rates as capability, memory, build times, and
  RMS-as-a-brag go stale and are machine/version-specific, so the docs state
  validation qualitatively ("validated on the Pi") instead. KEEP config values
  (param defaults, recommended `erc_rate`), inherent algorithmic timing (the
  inference-window default, prune intervals, the LED base-period math), and the
  stats-line output format. The calibrator still writes RMS into each
  `event_camera.yaml` header (a per-file record), just not quoted as a docs
  number. CLAUDE.md itself keeps its historical numbers (internal log, not student
  docs). See memory [[no-perf-numbers-in-docs]].

### Documentation
- The docs are a wiki/tutorial for students: assume no ROS "common
  knowledge" (terminal-per-node, sourcing, QoS are all explained or
  linked); state surprising-but-correct behavior right where it happens
  ("prints NOTHING when healthy", choppy blink preview, frozen-at-quiet).
- Pages SEQUENCE and hand off artifacts: tuning produces
  `~/my_params.yaml`; calibration consumes it and produces
  `event_camera.yaml`; rectification consumes both. Never show a command
  that ignores an earlier page's artifact.
- Neutral voice, no maintainer asides; every student stumble becomes a doc
  fix at the exact place of the stumble.

### Operations
- Stop ROS nodes with Ctrl+C (SIGINT), NEVER `kill -9`: SIGKILL leaves
  Fast DDS shared-memory debris in `/dev/shm` (including `sem.fastrtps_*`
  port mutexes) that silently poisons all later communication, and can
  knock the EVK4 off the USB bus. Recovery procedure: troubleshooting.md.
- Runtime machines consume this repo via `git pull` only — no credentials,
  no file pushes.
- Fail fast and loud; pin versions; no speculative features; no per-file
  license headers (root LICENSE covers the repo).

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
  - Services (`std_srvs/srv/Trigger`): `~/save_settings` (`save_biases` +
    `bias_file` REMOVED 2026-06-12 — params YAML is the single bias
    mechanism; .bias users copy values into the YAML)
    (`I_LL_Biases::save_to_file` / `cam_.save`); both fail clearly if no
    `settings` path was given at startup.
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
- **HAL plugin path (found 2026-06-22, a from-scratch-wipe finding):** the apt
  `openeb_vendor` SDK installs the Metavision HAL device plugins under its own
  prefix (`/opt/ros/$ROS_DISTRO/opt/openeb_vendor/lib/metavision/hal/plugins`)
  but ships NO env hook, so `MV_HAL_PLUGIN_PATH` is **unset** after sourcing —
  and the driver then fails to open the camera (`[HAL] no plugin found` ->
  `Error 101001: Camera not found`) on a CLEAN install. This had been masked for
  months by a manual export on the dev/lab boxes; only a full ROS-and-all wipe
  exposed it. Fix: `evk4.launch.py` now sets `MV_HAL_PLUGIN_PATH` to that dir
  (only if unset and the dir exists); `pipeline.launch.py` inherits it via its
  `IncludeLaunchDescription` of `evk4.launch.py`. A standalone `driver_node`
  run still needs the env set by hand.
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
  camera_info. **Bag replay (no camera):** the renderer times frames off the
  clock's `now()`, so replaying a bag renders NOTHING on wall time (target
  frame times sit ahead of the bag's old event stamps). Fix = sim time:
  `ros2 launch evk4_bringup replay.launch.py` (renderer alone,
  `use_sim_time:=true`, no driver) + `ros2 bag play <bag> --clock`. Validated
  2026-06-22; the sample bag is the `sample-data` GitHub release; docs in
  usage.md "No camera?".
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
- **Calibration tool (decision 2026-06-09; pattern 2026-06-10; headless
  2026-06-11):** our OWN guided calibrator `evk4_calibration` (ament_python,
  deps rclpy/sensor_msgs/cv_bridge/opencv/numpy) —
  `ros2 run evk4_calibration calibrate`. **HEADLESS, no GUI window** (the
  one-mechanism-per-job rule): publishes its annotated view on `~/overlay`
  (lazy — only when subscribed), watched in `rqt_image_view`; auto-captures
  distinct views (X/Y/Size/Skew coverage), and at READY auto-calibrates
  (`cv2.calibrateCamera` + `CALIB_FIX_K3`), writes the YAML (RMS in header),
  logs RMS, and EXITS; Ctrl+C aborts. No keys, no display needed on the
  camera host. Uses a **blinking ASYMMETRIC CIRCLE GRID**
  (`docs/circle_grid.html`, default `grid_size:=5x17`, drawn rotated so the
  bbox is exactly 16:9; rotation-independent detection verified
  synthetically): |B-R| polarity-contrast → GaussianBlur →
  SimpleBlobDetector (3 threshold passes, not the default 17) →
  half-res fast-path `findCirclesGrid(ASYMMETRIC|CLUSTERING)` + full-res
  centroid refinement on hit, throttled to new frames, `cv2.setNumThreads(2)`
  — all on a worker thread. Overlay architecture (2026-06-11): the live view
  is DECOUPLED from detection — the subscription callback republishes every
  frame immediately (full-res; bars/status), the worker republishes analyzed
  frames with markers; BOTH ends use depth-1 QoS (deeper queues = standing
  latency). `debug_timing` param logs per-cycle stage breakdown + frame age. Performance lesson (2026-06-11): an unthrottled full-res
  blob detector starved the whole Pi pipeline; detection must have a cheap
  fast-path. **Checkerboards were tried and ABANDONED** (hardware experience
  2026-06-10): saddle-point corners drift on speckle/gappy event edges even
  with blur — circle centroids average hundreds of events and are robust
  (same conclusion as E-Calib and eKalibr; cited in calibration.md).
  **Deliberately DROPPED E2VID/e2calib** (user decision): deep-learning
  reconstruction + Kalibr are too heavy / not "just works", esp. on Pi — do
  NOT add them back. Circle mode HW-VALIDATED end-to-end (2026-06-11, user
  ran the one-command session): RMS 0.895 px / 20 views, principal point
  (621,370) vs image center (640,360), fx/fy match to 0.07%, max
  undistortion displacement 5.7 px (the EVK4 kit 8mm lens is nearly
  distortion-free — side-by-side raw/rect looking identical is correct).
  Note on Metavision's own Calibration module (corrected 2026-06-17): it is NOT
  x86-only -- `libmetavision_sdk_calibration.so` DID build in our ARM SDK 5.3.1
  source build (it needs only OpenCV calib3d + Ceres, not Sophus/cv3d). We still
  prefer our open OpenCV `evk4_calibration` for INTRINSICS (no SDK dep, integrated,
  validated). Where the SDK module would actually help is EXTRINSIC / multi-camera
  (stereo -> 3D / mocap) calibration -- and it supports active-LED marker boards,
  tying into led_tracking. That needs cv3d (now BUILT 2026-06-18 -- USE_SOPHUS=ON
  via `ros-$ROS_DISTRO-sophus`) + a 2nd camera; with cv3d solved, the only blocker
  left for stereo/mocap is the second camera (the experimental stereo tier).
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
      live biases, save_settings, honored frame_id. It is the
      production driver in `evk4.launch.py`; whole pipeline (renderer,
      examples, calibration camera_info, bag) re-validated on the Pi. Install
      switched to `openeb_vendor`; docs truthed up. (Phases 0–3; details in
      memory `sdk-rewrite-decision`.)
- [x] **x86 re-validation of the OpenEB driver (2026-06-13):** documented
      student flow run COLD on the lab PC (different EVK4, serial 00051701) —
      script install (patch auto-applied, idempotent), overlay build, smoke
      test, tuned config, calibration (RMS 0.635 px / 20 views), and
      rectification, all working, no lag. Surfaced + fixed: script-vs-manual
      either/or wording, -Wcomment in the C++ example, guarded bashrc
      deps-source line, and bashrc now also sources the overlay so new
      terminals run `evk4_*` immediately.
- [x] **SDK Pro layer — `evk4_sdk_advanced`, sparse optical flow (2026-06-16):**
      source-built Metavision SDK 5.3.1 on the Pi 5 (ARM, ~22 min, lean config
      Torch/Sophus/Studio off) — this folds in the "SDK Pro feasibility spike"
      (builds + runs on ARM, validated). Built `evk4_sdk_advanced::OpticalFlow`:
      decodes `/event_camera/events` -> `vector<EventCD>`, runs the SDK's
      `SparseOpticalFlowAlgorithm` -> `/event_camera/optical_flow_image` (renamed
      from `flow_image` 2026-06-16 to match the uniform `<pipeline>_image`
      convention), with NO SDK
      edits (consumed via `process_events`). Real-time on the Pi (21 ms latency,
      30 fps) after a two-thread, on-demand-frame-gen rewrite; `my_params`/ERC
      govern it; one-command launch (no setup_env.sh — SDK lib path baked in).
      Docs: `docs/sdk/` (access, install, optical_flow) + a "getting SDK access"
      tutorial written from the user's live run. Details: memory
      `sdk-pro-integration`.

- [x] **Shared harness + object tracking (2026-06-16):** refactored the
      threading/decode into a reusable base `evk4_sdk_advanced::EventVisionNode`
      (include/.../event_vision_node.hpp) -- each pipeline is now ~3 hooks
      (processEvents/stageResults/renderFrame) + a launch. Added the second
      pipeline, object tracking (`TrackingAlgorithm` -> labeled bounding boxes
      -> /event_camera/tracking_image), both bag-validated on the Pi. Caught +
      fixed two real bugs by inspecting pixels: split-lock staging desync (flow
      arrows vanished) and a destructor-order use-after-free (tracking hung on
      SIGINT, needed -9). Docs: docs/sdk/pipelines.md.

- [x] **Five more model-free pipelines (2026-06-16):** on the same
      `EventVisionNode` harness — `dense_flow` (TripletMatchingFlow ->
      DenseColorMap; added `display_max_flow` to decouple color scale from the
      matching ceiling so slow scenes light up), `spatter`
      (SpatterTrackerAlgorithm, two-step `process_events`+results, id-labeled
      boxes), `counting` (CountingAlgorithm + CountingDrawingHelper, line
      counter + running-count overlay), `frequency` (FrequencyMapAsyncAlgorithm
      -> HeatMap; seed an empty map in onInit so it ALWAYS publishes the
      colorbar frame, else a non-periodic scene shows nothing), `led_tracking`
      (two-stage ModulatedLightDetector -> ActiveLEDTracker — needs modulated
      active-LED markers). ONE generic `pipeline.launch.py`
      (`pipeline:=<name>`) is now the single launch for ALL 7 pipelines: the two
      originals (optical_flow, tracking) were folded in and their dedicated
      `optical_flow.launch.py`/`tracking.launch.py` REMOVED (one-mechanism-per-job);
      optical_flow's topic renamed `flow_image` -> `optical_flow_image` so every
      pipeline follows `<name>_image`. Pipeline-specific params (tracking
      min_size/max_size, etc.) are node defaults overridden with `--ros-args -p`.
      dense_flow/spatter/counting CONTENT-validated by bag
      replay + pixel inspection. **frequency VALIDATED live 2026-06-17** (fan
      blade-pass ~60-70 Hz); **led_tracking VALIDATED live 2026-06-17** too -- a
      Raspberry Pi GPIO drove an LED with the modulated-light code (a homemade
      active marker), the node decoded ID 146 and tracked it (green circle +
      "146"). ALL 7 SDK pipelines now validated. Bag-validation gotchas
      caught: SIGINT to the `ros2 run` launcher does NOT forward to the node
      binary -> use `setsid` + `kill -INT -PGID`; the LED tracker consumes
      `EventSourceId`, not `EventCD`. Docs: docs/sdk/pipelines.md.

- [x] **Node-drops-events investigation (2026-06-16), overload-warning fix
      REVERTED (2026-06-17):** the frequency map went black on a high-rate flicker
      NOT from the SDK but because `EventVisionNode` runs decode + the algorithm
      inline on the subscription thread (~3 Mev/s ceiling on a Pi; decode alone
      ~7.5 Mev/s) and best-effort QoS then SILENTLY drops events, shredding the
      per-pixel periodicity frequency needs. Proven with a captured flicker bag:
      full rate drops ~79% -> ~0 detections; fed within budget -> ~2200 px
      (matches the algorithm offline at 12 Mev/s). A fix was built (an overload
      WARNING via `warnOnOverload()`/`checkOverload`, a `rendersEvents()`
      staging-skip, an on-image status overlay) then **REVERTED at the user's
      request** after an A/B showed the pre-fix code detects the fan identically:
      the fix only matters for bright HIGH-rate full-frame sources, and the
      validated use cases (fan, etc.) are low-rate, so it was dead weight. The
      drop FACT stands (high rate -> silent black -> cap erc_rate); we just don't
      warn. The real lever for high rates is ERC capping (ERC's controlled
      on-sensor drop preserves periodicity; transport drops don't).
      A browser flicker-test page was tried then REMOVED (the fullscreen strobe
      caused temporary LCD image retention on the user's laptop — do not re-add a
      strobe tool; for vibration tests use a FAN or mains lamp, never a screen).
      Frequency then VALIDATED live 2026-06-17. The two real blockers to live
      detection turned out to be OPTICS and the rate budget, not code: an
      out-of-focus / dim scene gives only sensor noise (no periodicity) -> open
      the aperture (f/2), focus until edges are crisp (check on image_raw), get
      CLOSE so the source fills the frame; and cap erc_rate (~2-3 Mev/s) so the
      node doesn't drop. A fan is marginal (fast tips blur) -> low speed, look
      straight down at the blades from close.

Next (user-ordered):
- [x] **All 7 SDK pipelines validated (2026-06-17).** `led_tracking` was the
      last: validated with a homemade marker -- a Pi GPIO blinking an LED with the
      modulated-light code (`docs/sdk/led_marker.c`). Encoding (from the SDK
      gtest): a blink = an LED rising edge; gap between rising edges in multiples
      of base period p encodes a symbol -- 2p=bit0, 3p=bit1, 4p=start; ID = 8 bits
      LSB-first, framed by starts. For a Pi-driven (slow) marker use base 5 ms +
      `inactivity_period_us` > the blink gap (Linux can't do the real 200 us
      reliably; the code re-syncs on every start). Decoded ID 146, tracked it.
- [x] **PSM + jet_monitoring added (2026-06-17) -> 9 pipelines.** Particle Size
      Monitoring (`PsmAlgorithm`: count + size of objects crossing N horizontal
      lines; move-only `LineParticleTrackingOutput` -> std::move across threads)
      and Jet Monitoring (`JetMonitoringAlgorithm`: count dispensed jets via
      event-rate peaks in a ROI). Both analytics-module, on the harness. Both
      VALIDATED LIVE on the Pi (2026-06-17): PSM ran the count up (Counter=231)
      with line clusters + a particle trajectory as objects passed vertically; jet
      counted fast bursts flicked through the center ROI (Counter=2, ~590 kev/s
      bursts vs the 50 kev/s th_up, no false counts at the ~30 kev/s baseline).
      ALL 9 SDK pipelines now live-validated. This
      covers every 2D model-free analytics app the SDK lists. The remaining SDK
      apps (ArUco, model-3D, active markers, stereo) are all `cv3d` -- NOT built at
      this point (USE_SOPHUS=OFF), documented then as the untested 3D tier.
      (SUPERSEDED 2026-06-18: cv3d built + `edgelet` added -- see below.)
- [x] **`undistortion` pipeline added (2026-06-17) -> 10 pipelines.** Event-level
      lens rectification (the event-stream counterpart of image_proc, which only
      rectifies image_raw). CV-module `PinholeCameraModel` + `CameraGeometry`,
      built from the SAME ROS `camera_info` YAML evk4_calibration produces
      (`info.k`->K[9], `info.d`->D[5] plumb_bob -> the SDK ctor takes K,D directly,
      NO JSON conversion). Precomputes a distorted->undistorted pixel LUT in onInit
      (img_to_undist_norm + undist_norm_to_undist_img per cell; ~few-second build),
      then remaps each event and reuses OnDemandFrameGenerationAlgorithm for the
      stock event-frame look. `calibration_url` is a launch arg (node param);
      VALIDATED LIVE on the Pi with a synthetic-distortion camera_info file. Build
      gotcha caught: `calibration_url` leaked into the included evk4.launch.py
      (launch configs inherit into IncludeLaunchDescription) -> the driver tried to
      start its CameraInfoPublisher (needs viz) and errored; fixed by passing
      `calibration_url: ''` explicitly to the driver include. Linked from
      calibration.md ("for events, use the SDK undistortion pipeline"). This was
      the one genuine classical gap from the SDK-catalog audit; noise_filtering/
      data_rate are redundant (driver does STC on-sensor; ERC + topic hz cover rate).
- [x] **ML inference tier (`evk4_sdk_advanced`, x86 + GPU) — 3 pipelines + a
      throughput fix (2026-06-18).** Source-built the SDK with Torch/CUDA (LibTorch
      cu126) on the lab PC; added `gesture` (`convRNN_chifoumi`), `detection`
      (`red_event_cube` SSD + DataAssociation tracking), `flow_inference`
      (`model_flow`) on a shared `MlVisionNode` base (model load + EventPreprocessor
      + slicer + `model_->infer` every `delta_t_us`). Gated on Torch + the SDK `ml`
      module (the Pi's lean build skips them). **Fixed a 300x throughput bug:** the
      ~50 ms GPU inference ran INLINE on the EventVisionNode subscription thread and
      saturated it (detection fell to ~0.07 fps; the `0.00 Mev/s` reading was a
      saturation artifact -- the thread never reached the event counter, NOT zero
      events). Moved inference to its OWN thread (sub thread only decodes + enqueues,
      bounded drop-oldest so the model sees recent events; frame thread renders at
      fps with the latest results; `extractResults` under `mutex()`). Detection
      0.07 -> 21 fps, gesture 27, flow 25, all at the 30 fps node rate, clean SIGINT.
      Memory `ml-inference-needs-own-thread`.
- [x] **cv3d tier + `edgelet` pipeline -> 11 model-free/CV pipelines (2026-06-18).**
      Enabled the SDK `cv3d` module (`ros-$ROS_DISTRO-sophus` apt + reconfigure the
      SDK with `-DUSE_SOPHUS=ON`; our package passes `-DSophus_DIR=...` for cv3d's
      transitive Sophus dep). Added `edgelet` (Edgelet2dDetection/Tracking on a time
      surface, plain EventVisionNode harness -- cheap CV algo, no inference thread),
      VALIDATED on the lab PC (30 fps, ~2 Mev/s). **STOPPED at edgelet** (user
      decision): the rest of the cv3d/3D apps are gated NOT on the build (solved) but
      on -- ArUco: BLOCKED on licensing (its marker detection is proprietary SDK
      *sample* code `aruco_nano.h`, not a library API -> can't vendor into the Apache
      repo; OpenCV `cv::aruco` is the substitute); model-3D: needs a CAD model +
      pose; active markers: need a marker board; stereo: a 2nd camera. Documented as
      the gated tier in pipelines.md + install.md (cv3d build steps). Memory
      `cv3d-tier`.
- [ ] Docs media pass — GIFs of the tuning experiments + rectified view
      (calibration demo done; tuned_stream_demo.gif still 18 MB, re-shrink).
- [ ] Upstream PR for the renderer backlog cap (the vendored patch is the
      stopgap; an accepted PR retires it).
- [ ] OPEN: fast-motion moving-edge trail — sensor-side (persists at 1000 fps);
      levers to try are trail_filter_threshold down, bias_refr up, bias_fo.

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
- Verify package XML/builds with `colcon build` where Jazzy is available;
  otherwise validate `package.xml` as XML and note the limitation.
- Reference upstream docs rather than duplicating them.

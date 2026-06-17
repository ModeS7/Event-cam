# Metavision SDK Pro — the advanced layer (optional)

An **opt-in** layer that runs **closed-source Metavision SDK** computer-vision
algorithms on the EVK4 event stream. The base repo (driver, renderer, examples,
calibration) is built on **OpenEB** and needs none of this — read on only if you
want SDK algorithms like optical flow, tracking, or vibration analysis.

It is self-contained: the package here (`evk4_sdk_advanced`) **subscribes to
`/event_camera/events`** — the same topic everything else uses — decodes it, and
feeds the events to the SDK through its camera-independent `process_events` API.
The SDK touches the repo in exactly one optional package, behind the normal event
stream. If you never install the SDK, a normal `colcon build` simply skips it.

> **Detailed per-pipeline reference: [pipelines.md](pipelines.md)** — params,
> behavior, tuning, validation, and the LED-marker test rig live there. This page
> is the brief overview + quick start.

## Setup (once)

1. [access.md](access.md) — get a Prophesee account and an identity token (the
   gated credential; free for EVK4 owners).
2. [install.md](install.md) — install the SDK: `apt` on x86_64, or a source
   build on ARM (Raspberry Pi / Jetson).
3. **Build the package** (it's skipped unless you point CMake at the SDK):
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select evk4_sdk_advanced --cmake-args \
     -DMetavisionSDK_DIR=$HOME/metavision_src/openeb-5.3.1/build/generated/share/cmake/MetavisionSDKCMakePackagesFilesDir
   source install/setup.bash
   ```
   (On an **apt** SDK / x86, omit `-DMetavisionSDK_DIR`. The build captures the
   SDK's library path, so the launch sets it automatically — no `setup_env.sh`.)

## Quick start — run any of the seven

All pipelines share **one launch**; pick which with `pipeline:=`:

```bash
ros2 launch evk4_sdk_advanced pipeline.launch.py pipeline:=optical_flow \
    params_file:=$HOME/my_params.yaml
# view it (second terminal):
ros2 run rqt_image_view rqt_image_view /event_camera/optical_flow_image
```

Swap `pipeline:=<name>` and the topic `/event_camera/<name>_image`:

| `pipeline:=` | Shows | Point it at |
|---|---|---|
| `optical_flow` | event edges with **flow-vector arrows** | moving objects near the lens |
| `tracking` | **ID-labeled boxes** on moving objects | a few distinct movers |
| `dense_flow` | a **color flow field** (hue=direction, brightness=speed) | moving edges |
| `spatter` | **many small ID-boxes** at once | sparks / droplets / small fast movers |
| `counting` | a line + a **running count** | objects crossing the line |
| `frequency` | a **per-pixel vibration heat map** (Hz) | a fan / flickering light |
| `led_tracking` | a circle + the **decoded ID** | an active-LED marker |

`params_file:=$HOME/my_params.yaml` feeds your tuned driver setup (ERC cap,
biases, filters) — that, not the algorithm, is the main latency/CPU lever on a Pi.

**Two need a bit more than "point and run"** — see [pipelines.md](pipelines.md):
- **`frequency`** needs a genuinely *periodic* source and good optics (open the
  aperture, focus, light it) — most everyday scenes read black, correctly.
- **`led_tracking`** needs a *coded* marker, and the marker's `base_period_us` /
  `inactivity_period_us` passed via `node_params_file:=...`. The detail page shows
  how to build a test marker from a Raspberry Pi GPIO + an LED.

## Validation matrix

Honest evidence levels — **validated** = ran on hardware and recorded the result;
**expected** = inferred from an adjacent result, not run; **not viable** =
structurally blocked.

| Capability | x86_64 | Raspberry Pi 5 (ARM64) | Jetson (ARM64) |
|---|---|---|---|
| SDK build | expected (apt binaries) | **validated** (source, 2026-06-16) | expected (source) |
| Sparse optical flow | expected | **validated** (2026-06-16) | expected (from Pi) |
| Object tracking | expected | **validated** (2026-06-16) | expected (from Pi) |
| Dense flow / spatter / counting | expected | **validated** (bag + live, 2026-06-17) | expected (from Pi) |
| Frequency (vibration) | expected | **validated** (2026-06-17, fan ~60-70 Hz) | expected (from Pi) |
| Active-LED tracking | expected | **validated** (2026-06-17, Pi-GPIO LED marker, ID 146) | expected (from Pi) |
| ML detection (needs LibTorch) | expected (CPU/GPU) | **not viable** (no CUDA) | expected (CUDA, untested) |
| Stereo calibration | untested skeleton | untested skeleton | untested skeleton |

This project validates on a **Raspberry Pi 5** (the constrained, worst-case
target) and an x86 dev machine; Jetson is inference-only (same aarch64 + Ubuntu)
and **not** tested here. Never read "expected" as "works."

## Platform note

The SDK ships **prebuilt apt binaries for x86_64 only**. On ARM (Pi, Jetson) you
**build it from source** — a validated, ~22-minute procedure on a Pi 5
([install.md](install.md)). Everything downstream (`evk4_sdk_advanced`, the
launch, the results) is identical once the SDK is present. All seven pipelines
share one real-time harness (`event_vision_node.hpp`: decode `EventPacket` →
`vector<EventCD>` → an SDK algorithm → publish an image); adding one is three
small hooks plus a launch entry. ML detection and stereo calibration are the
heavier, experimental extensions still ahead (LibTorch / two cameras).

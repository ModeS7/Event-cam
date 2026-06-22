# Metavision SDK Pro — the advanced layer (optional)

An **opt-in** layer that runs **closed-source Metavision SDK** computer-vision
algorithms on the EVK4 event stream. The base repo (driver, renderer, examples,
calibration) is built on **OpenEB** and needs none of this — read on only if you
want SDK algorithms like optical flow, tracking, or vibration analysis.

It is self-contained: the package here (`evk4_sdk_advanced`) **subscribes to
`/event_camera/events`** — the same topic everything else uses — decodes it, and
feeds the events to the SDK through its camera-independent `process_events` API.
The SDK touches the repo in exactly one optional package, behind the normal event
stream. If you never install the SDK, a normal `colcon build` skips it.

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

## Quick start — run a pipeline

All pipelines share **one launch**; pick which with `pipeline:=`:

```bash
ros2 launch evk4_sdk_advanced pipeline.launch.py pipeline:=optical_flow \
    params_file:=$HOME/my_params.yaml
# view it (second terminal):
ros2 run rqt_image_view rqt_image_view /event_camera/optical_flow_image
```

Stop the pipeline with Ctrl+C — never `kill -9` (it leaves DDS shared-memory
debris; recovery is in [troubleshooting.md](../troubleshooting.md)).

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
| `psm` | **count + size** of objects crossing lines | parts on a conveyor / falling particles |
| `jet_monitoring` | **count of dispensed jets** in a ROI | a dispensing nozzle |
| `undistortion` | the **event stream rectified** for lens distortion | any scene (needs a `calibration_url`) |

`params_file:=$HOME/my_params.yaml` feeds your tuned driver setup (event-rate
cap, biases, filters — see [tuning.md](../tuning.md)) — that, not the algorithm,
is the main latency/CPU lever on a Pi.

**Three need more setup than a launch command** — see [pipelines.md](pipelines.md):
- **`frequency`** needs a genuinely *periodic* source and good optics (open the
  aperture, focus, light it) — most everyday scenes display black, which is correct.
- **`led_tracking`** needs a *coded* marker, and the marker's `base_period_us` /
  `inactivity_period_us` passed via `node_params_file:=...`. The detail page shows
  how to build a test marker from a Raspberry Pi GPIO + an LED.
- **`undistortion`** needs `calibration_url:=<your event_camera.yaml>` (the file
  from [calibration](../calibration.md)); it refuses to start without it.

The ten above are the **model-free** pipelines — they build with the base SDK and
run anywhere. Two more tiers launch the same way but need an extra SDK build
(see [install.md](install.md), [pipelines.md](pipelines.md)):

- **`edgelet`** (cv3d tier) — 2D edge-segment tracking; needs the SDK rebuilt with
  `-DUSE_SOPHUS=ON` (no GPU).
- **`gesture`, `detection`, `flow_inference`** (ML/GPU tier) — pretrained neural
  nets; need LibTorch + the SDK `ml` module + a GPU (x86).

## Platform note

The SDK ships **prebuilt apt binaries for x86_64 only**. On ARM (Pi, Jetson) you
**build it from source** — a validated, ~22-minute procedure on a Pi 5
([install.md](install.md)). Everything downstream (`evk4_sdk_advanced`, the
launch, the results) is identical once the SDK is present. The model-free and
cv3d pipelines share one real-time harness (`event_vision_node.hpp`: decode
`EventPacket` → `vector<Metavision::EventCD>` → an SDK algorithm → publish an
image); the ML pipelines extend it (`ml_vision_node.hpp`) with a dedicated
inference thread so the ~50 ms GPU model never stalls event ingestion. Adding a
pipeline is a few small hooks plus a launch entry —
see [extending.md](extending.md).

Validated on a **Raspberry Pi 5** (ARM source build) and an x86 dev box;
**Jetson is untested** (same aarch64). The ML pipelines need CUDA, so they won't
run on the Pi (the **ML/GPU tier**). The `edgelet` pipeline is the **cv3d tier** — it
needs the SDK rebuilt with `-DUSE_SOPHUS=ON` ([install.md](install.md)) but no
GPU. The remaining 3D apps (ArUco, model-3D, active markers, stereo) stay gated on
a marker / model / second camera. See [pipelines.md](pipelines.md) for the
per-pipeline behavior, tuning, and gating.

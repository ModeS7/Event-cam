# SDK pipelines — detailed reference

The deep reference for the ten `evk4_sdk_advanced` pipelines: parameters,
behavior, tuning, and validation. For the brief overview + the one-command
quick-start, see [README.md](README.md); for setup, [access.md](access.md) and
[install.md](install.md).

Jump to: [optical_flow](#sparse-optical-flow--optical_flow) ·
[tracking](#object-tracking--tracking) ·
[dense_flow](#dense-optical-flow--dense_flow) ·
[spatter](#particle--spatter-tracking--spatter) ·
[counting](#object-counting--counting) ·
[frequency](#vibration-frequency--frequency) ·
[led_tracking](#active-led-tracking--led_tracking) ·
[psm](#particle-size-monitoring--psm) ·
[jet_monitoring](#jet-monitoring--jet_monitoring) ·
[undistortion](#undistortion--undistortion) ·
[ML pipelines (GPU)](#ml-inference-pipelines-gpu) ·
[3D apps (untested)](#using-the-untested-3d-applications)

## How they all work (the shared harness)

Every pipeline is one SDK algorithm wrapped as a ROS node. It decodes
`EventPacket` → `vector<Metavision::EventCD>` and feeds the SDK over its
camera-independent `process_events` API — the SDK is consumed, never modified. They share one real-time harness (`event_vision_node.hpp`) with **two
threads**: the subscription thread decodes and runs the algorithm *incrementally*
per packet (SDK algorithms are streaming and go super-linear on large batches); a
frame thread, paced to wall-clock `fps`, renders one newest frame on demand and
publishes it. A mutex guards only a cheap buffer swap, so rendering never stalls
ingestion — that split is what keeps latency low. Adding a pipeline is three small
hooks (`processEvents` / `stageResults` / `renderFrame`) plus a launch entry.

Common launch args (all pipelines): `pipeline`, `params_file` (your driver
`~/my_params.yaml`), `camera_name`, `serial`, `frame_id`, `fps` (default 30),
`debug_timing`, and `node_params_file` (a YAML of pipeline-specific node params).
Every pipeline also has `accumulation_time_us` (default 10000 = 10 ms, the event
window drawn per frame). Each publishes `/event_camera/<pipeline>_image`
(`sensor_msgs/Image`, bgr8).

---

## Sparse optical flow — `optical_flow`

Event edges with **flow-vector arrows** overlaid — each tracked feature's
direction and speed, color-coded. Uses `SparseOpticalFlowAlgorithm`.

**Behavior:**
- **Needs close, definite motion.** The tuned config (STC trail filter) keeps the
  stream clean and *sparse*, so distant hand-waving barely registers.
- **Too much motion makes vectors *disappear* — intended.** Sparse flow only emits
  a vector where it can confidently match a distinct feature between moments;
  overwhelm it and it goes quiet rather than guess. Moderate motion works best.
- A quiet scene holds the last frame (no events → no update), which is correct.

**Validated on the Pi:** ~21 ms median latency (camera → image), 50 ms
p99; 30 fps. Dense scenes are flow-bound on the Pi — cost scales with *feature
count* (~6.7 Mev/s gentle, ~0.5 Mev/s dense). When a scene exceeds what the Pi can
flow live, it samples the trackable features and drops the surplus — which does
not degrade the vectors. x86/Jetson process the full rate; it's the Pi's compute
floor, not a bug.

**Reproducible testing with a bag** (live motion is an inconsistent stimulus):
```bash
ros2 launch evk4_bringup evk4.launch.py params_file:=$HOME/my_params.yaml viz:=false &
ros2 bag record -o ~/flow_demo /event_camera/events     # Ctrl+C after ~15 s of motion
ros2 bag play ~/flow_demo &
ros2 run evk4_sdk_advanced optical_flow --ros-args \
  -r events:=/event_camera/events -r optical_flow_image:=/event_camera/optical_flow_image
```
(Latency must be checked live — a bag carries the original timestamps; the bag is
for throughput / frame rate / visual quality.)

---

## Object tracking — `tracking`

A labeled **bounding box** on each moving object. Uses `TrackingAlgorithm`.

| Node param | Default | Description |
|---|---|---|
| `min_size` / `max_size` | `10` / `300` | Min/max tracked-object size (px) |

**Behavior:**
- `min_size` / `max_size` set what counts as an object; defaults suit hand-sized
  objects close to the lens — widen for very small or large movers.
- On very dense scenes it spawns many short-lived boxes; the tuned (ERC-capped,
  STC) stream keeps that manageable.

Override the size range via `node_params_file:=...` (a `/**:` YAML), or run the
node directly with `--ros-args -p min_size:=5 -p max_size:=500`.

**How it differs from flow:** `stageResults` merges the latest box per object id
(the tracker emits many updates per object — keep one, drop ones unseen for
100 ms); `renderFrame` draws the event image, then `draw_tracking_results`.

---

## Dense optical flow — `dense_flow`

A full **color flow field** (hue = direction, brightness = speed), vs the sparse
arrows above. Uses `TripletMatchingFlowAlgorithm` → `DenseFlowFrameGeneratorAlgorithm`.

| Node param | Default | Description |
|---|---|---|
| `radius` | `3.0` | Spatial match search radius (px) |
| `max_flow` | `1000.0` | Matching ceiling (px/s) |
| `display_max_flow` | `300.0` | Color full-scale speed (px/s) — lower to brighten slow scenes |

Coverage follows moving edges (triplet-matching flow is semi-dense, so quiet
regions stay dark). Brightness is each pixel's speed normalized to
`display_max_flow`. **Validated on the Pi.**

---

## Particle / spatter tracking — `spatter`

Tracks **many small fast movers at once** (sparks, droplets, particles), each a
small ID-labeled box. Uses `SpatterTrackerAlgorithm`.

| Node param | Default | Description |
|---|---|---|
| `cell_size` | `7` | Tracking cell size (px); smaller = finer particles |

Keeps the latest box per cluster id, drops ones unseen for 100 ms. **Validated on
the Pi.**

---

## Object counting — `counting`

Counts objects **crossing a horizontal line** (e.g. parts on a conveyor) and
overlays the running count + timestamp. Uses `CountingAlgorithm`.

| Node param | Default | Description |
|---|---|---|
| `line_row` | `360` | Image row of the counting line (px from top) |
| `cluster_ths` | `5` | Minimum cluster size to count |

Put the line where objects cross, and tune the stream (ERC cap, STC) so each
object is one clean cluster — on a cluttered scene the count climbs fast (it's
meant for discrete objects). **Validated on the Pi.**

---

## Vibration frequency — `frequency`

Estimates the **blink / vibration frequency at each pixel** and renders a JET heat
map with a Hz colorbar. Uses `FrequencyMapAsyncAlgorithm` → `HeatMapFrameGeneratorAlgorithm`.

| Node param | Default | Description |
|---|---|---|
| `min_freq` / `max_freq` | `10.0` / `150.0` | Frequency band shown (Hz); also the colorbar range |
| `filter_length` | `7` | Stable periods needed before a pixel registers |
| `diff_thresh_us` | `1500` | Allowed ± per-period difference (µs) |

**The frame is black where no periodic motion is detected** — correct, *not a
broken node*. Most everyday scenes (hand motion, walking) produce nothing. To see
it light up, point it at a genuinely periodic source: a **spinning fan**, a
**mains-powered light** (100 Hz / 120 Hz, both in range), a vibrating tool.

> **Recommended for reliable detection (in order of impact):**
> 1. **Open the aperture all the way (f/2)** — most light onto the sensor.
> 2. **Light the object brightly** — a lamp or **phone flashlight** straight at
>    it. More light = stronger moving edges = far more events (this alone took a
>    test fan from a few flickering spots to a solid detection).
> 3. **Focus precisely** — check on `image_raw` first; uniform speckle = out of
>    focus → only sensor noise, which has no per-pixel periodicity.

A **fan** is marginal (fast tips blur — only mid-blade pixels lock); slower speed
and looking straight down at the blades is steadiest.

**Keep the event rate within budget.** Frequency needs *every* event per pixel, so
dropped events break it. If the rate exceeds what the node can process in real
time (~3 Mev/s on a Pi 5), the transport silently drops events and the map goes
**black even on a visibly flickering scene** — a bright full-frame strobe
(~8 Mev/s) does exactly this. Fix: **cap `erc_rate`** in your params (e.g.
`erc_rate: 3000000`) — ERC drops on the sensor in a controlled way that preserves
periodicity, unlike the transport's random whole-packet drops — or restrict the
field of view with an `roi`.

**Validated on the Pi:** lens at f/2, camera close above a desk fan →
locked the blade-pass at ~60–70 Hz. The two things that decide success are
**optics** (focus + light) and **staying within the event-rate budget** — not the
algorithm (a synthetic 100/50 Hz input maps to exactly that frequency).

---

## Active-LED tracking — `led_tracking`

Tracks **active LED markers** — LEDs that transmit a numeric **ID** by blinking a
coded pattern — drawing a circle + decoded ID on each. The event-camera answer to
an ArUco fiducial, and the basis of active-marker motion capture (the ID resolves
marker correspondence — which detection is which marker). Two SDK stages chained:

```
EventCD ──▶ ModulatedLightDetectorAlgorithm ──▶ EventSourceId ──▶ ActiveLEDTrackerAlgorithm ──▶ tracks
            decode each LED's ID from blink       per-event           one circle + ID per LED
            timing                                 source id
```

A plain (steady or fixed-rate) LED produces **no valid ID** — nothing is tracked.

### The encoding

A blink is an LED **rising edge**; the gap between consecutive rising edges, in
multiples of the **base period** `p` (= `base_period_us`), encodes a symbol:

| Gap | Symbol |
|---|---|
| `2p` | bit `0` |
| `3p` | bit `1` |
| `4p` | start / sync |

An ID is **8 bits, LSB-first**, framed by start symbols. E.g. ID 146 =
`10010010` → bits `0,1,0,0,1,0,0,1` → gaps `2p,3p,2p,2p,3p,2p,2p,3p` then a `4p`
start. OFF edges and sub-period noise are ignored.

### Build a test marker on a Raspberry Pi

No commercial marker needed — a GPIO-driven LED works.

**1. Wire an LED:**
```
GPIO17 (header pin 11) ──[ 220–330 Ω ]──►|── GND (pin 9)
                                         LED   long leg = +,  short leg = –
```
The resistor is mandatory (GPIO is 3.3 V); **150 Ω** is brighter (~9 mA, under the
~16 mA limit) and helps the camera (see tuning). A backwards LED will not light.

**2. Allow GPIO access** (Pi 5 = `/dev/gpiochip4`, `pinctrl-rp1`, group `dialout`):
```bash
sudo usermod -aG dialout $USER     # new session to take effect
```

**3. Build + run the marker** ([`led_marker.c`](led_marker.c)):
```bash
gcc -O2 -o led_marker docs/sdk/led_marker.c
./led_marker 146 500 17            # id=146  base=500us  GPIO17  [flash_us]
```
It pins to a core and busy-waits, so timing stays clean. The LED looks steadily-on
to the eye but the camera sees every blink. Any ID 0–255; different markers use
different IDs so several can be tracked at once.

### Run the tracker

The Pi marker is slow (Linux can't clock the real 200 µs reliably), so two params
must match it — pass them with `node_params_file`:
```bash
cat > /tmp/led.yaml <<'YAML'
/**:
  ros__parameters:
    base_period_us: 500          # = the marker's base
    inactivity_period_us: 2000   # > the blink gap, so the track survives between blinks
YAML
ros2 launch evk4_sdk_advanced pipeline.launch.py pipeline:=led_tracking \
    params_file:=$HOME/my_params.yaml node_params_file:=/tmp/led.yaml
ros2 run rqt_image_view rqt_image_view /event_camera/led_tracking_image
```
Point the EVK4 at the LED (close, focused) → a **green circle + "146"** locks on.
A real 200 µs hardware marker needs no override — plain `pipeline:=led_tracking`.

| Node param | Default | Description |
|---|---|---|
| `base_period_us` | `200` | Base blink period — **must equal the marker's base** |
| `inactivity_period_us` | `1000` | Drop a track after this; **must exceed the blink gap** |
| `num_bits` | `8` | Bits per ID |
| `tolerance` | `0.1` | Allowed ±fraction per measured gap |
| `radius` | `10.0` | Event-to-track association radius (px) |

### Tuning (validated on the Pi)

- **Base period is the master knob.** A word ≈ 9 blinks, so at 5 ms base it takes
  ~115 ms — the LED must stay put that long to decode, so fast motion smears the
  code and drops. Shrink the base for motion: 5 ms → 500 µs cuts the word to
  ~11 ms (~10× better). ~500 µs is the Pi's reliable floor.
- **A faster code needs a brighter blink.** Shrinking the
  base shrinks the LED on-time → less light per blink → fewer events, especially
  mid-motion. Add light back: raise the marker's **flash** (4th arg) and/or the
  LED current (smaller resistor). 200 µs only became usable with more flash:
  `./led_marker 146 200 17 300`.
- **Reflections decode the same ID** — a reflection blinks the identical code, so
  the detector reads the same ID from it. Keep `radius` small and **matte / angle
  the scene** so no shiny surface throws the LED back at the camera.

Rule of thumb: **small base + bright blink + small radius** = fast motion + fewer
reflection grabs; the Pi marker tops out around 200–500 µs. A microcontroller
(tight timing) + a transistor-driven LED (more current) is what gets past that —
i.e. a commercial active marker.

### Where it goes: 3D / motion capture

One camera = **2D position + ID** per marker. 3D needs the same marker across
**≥2 calibrated cameras** (or single-camera PnP for a known rigid constellation).
IR LEDs (invisible, filterable) + unique IDs = outside-in mocap, and the
modulation **rejects steady light** (room lights and sunlight never
decode). Multi-camera extrinsic calibration is the experimental tier (needs a
second EVK4); the SDK's calibration module even supports active-LED marker boards.

---

## Particle Size Monitoring — `psm`

Counts objects **crossing a set of horizontal lines** and estimates each
particle's **size** — conveyor / channel QC of fast-moving objects. Uses
`PsmAlgorithm` (line-cluster detection + particle tracking across lines).

| Node param | Default | Description |
|---|---|---|
| `min_y` / `max_y` | `150` / `570` | Y range over which the lines are placed |
| `num_lines` | `6` | Number of line trackers between min_y and max_y |
| `cluster_ths` | `3` | Min cluster width (px) along a line; below = noise |
| `num_clusters_ths` | `4` | Min cluster measurements for a real particle |
| `precision_time_us` | `1000` | Async processing period (sets the line bitset buffer) |
| `dt_first_match_us` | `10000` | Max time to match a particle across two lines |
| `is_going_up` | `false` | Motion direction (false = downward) |

Place the lines perpendicular to motion; each particle matched across enough lines
increments the count and gets a size estimate. **Validated on the Pi.**
Per-particle sizing needs distinct objects crossing cleanly (conveyor-style); hand
motion exercises the lines + count but not the sizing.

---

## Jet Monitoring — `jet_monitoring`

Detects and **counts jets (dispensed dots)** by spotting **event-rate peaks inside
a detection ROI**, overlaying the ROI, the running count, and the ROI event rate.
For monitoring dispensing processes. Uses `JetMonitoringAlgorithm`.

| Node param | Default | Description |
|---|---|---|
| `roi_x` / `roi_y` / `roi_w` / `roi_h` | `600` / `330` / `80` / `60` | Detection ROI (px) — where jets pass |
| `th_up_kevps` | `50` | Event rate (kev/s) above which a jet starts |
| `th_down_kevps` | `10` | Event rate below which a jet ends |
| `jet_accumulation_us` | `500` | Detection accumulation window (≈ just under the dispensing cycle) |
| `time_step_us` | `50` | Monitoring update period |

A jet = a sharp event-rate burst through the ROI as a dot is dispensed; each
increments the count. **Validated on the Pi.** A real
dispensing nozzle is the intended source; the ROI, running count, and live rate
all render. Defaults assume a fast cycle; raise `jet_accumulation_us` for slower
dispensing, and tune `th_up_kevps` to your stimulus.

---

## Undistortion — `undistortion`

Rectifies the **event stream** for lens distortion — the event-level counterpart
of `image_proc`, which can only rectify the rendered `image_raw`. It loads a
standard `camera_info` YAML (the file [evk4_calibration](../calibration.md)
produces), builds the SDK's pinhole `CameraGeometry` from the `K` matrix +
plumb_bob distortion, and remaps every event to its undistorted pixel position;
the output is the rectified event view. Uses `PinholeCameraModel` +
`CameraGeometry` (CV module).

| Node param | Default | Description |
|---|---|---|
| `calibration_url` | (required) | Path to the `camera_info` YAML (K + distortion). The node refuses to start without it. |

Pass the calibration as a launch arg (the other pipelines ignore it):
```bash
ros2 launch evk4_sdk_advanced pipeline.launch.py pipeline:=undistortion \
    params_file:=$HOME/my_params.yaml \
    calibration_url:=$HOME/ros2_ws/src/Event-cam/evk4_bringup/config/calibration/event_camera.yaml
```

The distorted-to-undistorted pixel map is precomputed once at startup (a one-time
cost of a few seconds over the full sensor, logged as "building undistortion
map"), so each event is a single lookup on the frame thread. **Validated on the
Pi**: a `camera_info` YAML in our format loads directly, the map builds, and the
remapped event view publishes. On the EVK4 kit's near-distortion-free 8 mm lens
the rectified frame looks nearly identical to the raw one — the displacement is
only a few pixels, which is correct (see [calibration.md](../calibration.md)). The
zero-distortion placeholder calibration makes this a pass-through.

---

## ML inference pipelines (GPU)

Three neural-network pipelines run the SDK's pretrained models on the event
stream. Unlike the model-free pipelines above they need **LibTorch + the SDK `ml`
module**, so they build and run only on the x86 + GPU setup ([install.md](install.md));
the Pi's lean build skips them. They share one base (`MlVisionNode`): decode
events → SDK `EventPreprocessor` → run the model on the GPU every `delta_t_us`
(default 50 ms) → draw the result.

| `pipeline:=` | Model | Shows |
|---|---|---|
| `gesture` | `convRNN_chifoumi` | Rock / Paper / Scissors label |
| `detection` | `red_event_cube` (automotive) | tracked, labeled boxes |
| `flow_inference` | `model_flow` | flow-vector arrows |

Each takes `model_path` (the `.ptjit`) and `gpu_id` (0 = first GPU, -1 = CPU) via
`node_params_file`:
```bash
cat > /tmp/ml.yaml <<'YAML'
/**:
  ros__parameters:
    model_path: <MODELS>/classification/convRNN_chifoumi/rnn_model_classifier.ptjit
    gpu_id: 0
YAML
ros2 launch evk4_sdk_advanced pipeline.launch.py pipeline:=gesture \
    params_file:=$HOME/my_params.yaml node_params_file:=/tmp/ml.yaml
```
publishing `/event_camera/<pipeline>_image`. **Validated on a 2× RTX 2080 Ti box**
(GPU-resident inference confirmed: 254 MiB / 618 MiB / 1242 MiB respectively).
Notes: `detection` is automotive — point it at driving footage, a desk yields no
boxes — and inference-heavy at full sensor resolution (a lower model input
resolution would speed it up); `gesture` and `flow_inference` run live on any
motion.

---

## Using the untested 3D applications

The SDK also offers **Active Marker 3D Tracking**, **ArUco Marker Tracking**, and
**Edgelet / Model 3D Tracking** (3D edges + fiducials for AR/VR). These are **not
built or tested in this repo** — they live in the SDK's `cv3d` module, which the
lean ARM build skips (`USE_SOPHUS=OFF`), and they are genuinely 3D (they need
camera intrinsics and produce a 6-DoF pose). To use them:

1. **Rebuild the SDK with `cv3d`** — re-run the source build ([install.md](install.md))
   with `-DUSE_SOPHUS=ON` and `cv3d` in the selected modules. Sophus is
   header-only; the rest of the build is unchanged.
2. **Get camera intrinsics** — calibrate first (`evk4_calibration`); 3D pose needs
   the camera matrix + distortion (a `camera_info` YAML).
3. **Provide the marker/model definition** — ArUco needs a dictionary; active
   markers need a marker-geometry JSON (the LED layout); model-3D needs the edge
   model.
4. **Wrap it like the others** — a new `EventVisionNode` subclass feeding the
   `cv3d` algorithm, publishing the pose and/or an annotated image. Multi-camera
   variants additionally need extrinsic (stereo) calibration — the experimental
   tier that needs a second EVK4.

# SDK pipelines — detailed reference

The deep reference for the seven `evk4_sdk_advanced` pipelines: parameters,
behavior, tuning, and validation. For the brief overview + the one-command
quick-start, see [README.md](README.md); for setup, [access.md](access.md) and
[install.md](install.md).

Jump to: [optical_flow](#sparse-optical-flow--optical_flow) ·
[tracking](#object-tracking--tracking) ·
[dense_flow](#dense-optical-flow--dense_flow) ·
[spatter](#particle--spatter-tracking--spatter) ·
[counting](#object-counting--counting) ·
[frequency](#vibration-frequency--frequency) ·
[led_tracking](#active-led-tracking--led_tracking)

## How they all work (the shared harness)

Every pipeline decodes `EventPacket` → `vector<Metavision::EventCD>` and feeds the
SDK over its camera-independent `process_events` API — the SDK is consumed, never
modified. They share one real-time harness (`event_vision_node.hpp`) with **two
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

![Sparse optical flow: event edges with flow-vector arrows](images/optical_flow.png)

**Behavior (not what you'd guess):**
- **Needs close, definite motion.** The tuned config (STC trail filter) keeps the
  stream clean and *sparse*, so distant hand-waving barely registers.
- **Too much motion makes vectors *disappear* — intended.** Sparse flow only emits
  a vector where it can confidently match a distinct feature between moments;
  overwhelm it and it goes quiet rather than guess. Sweet spot: moderate motion.
- A quiet scene holds the last frame (no events → no update), which is correct.

**Validated (Pi 5, 2026-06-16):** ~21 ms median latency (camera → image), 50 ms
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

![Object tracking: labeled bounding boxes on moving objects](images/tracking.png)

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

![Dense optical flow color field](images/dense_flow.png)

| Node param | Default | Description |
|---|---|---|
| `radius` | `3.0` | Spatial match search radius (px) |
| `max_flow` | `1000.0` | Matching ceiling (px/s) |
| `display_max_flow` | `300.0` | Color full-scale speed (px/s) — lower to brighten slow scenes |

Coverage follows moving edges (triplet-matching flow is semi-dense, so quiet
regions stay dark). Brightness is each pixel's speed normalized to
`display_max_flow`. **Validated** on the Pi (bag 2026-06-16 + live 2026-06-17).

---

## Particle / spatter tracking — `spatter`

Tracks **many small fast movers at once** (sparks, droplets, particles), each a
small ID-labeled box. Uses `SpatterTrackerAlgorithm`.

![Spatter tracking: many small ID-labeled boxes](images/spatter.png)

| Node param | Default | Description |
|---|---|---|
| `cell_size` | `7` | Tracking cell size (px); smaller = finer particles |

Keeps the latest box per cluster id, drops ones unseen for 100 ms. **Validated**
on the Pi (bag 2026-06-16 + live 2026-06-17).

---

## Object counting — `counting`

Counts objects **crossing a horizontal line** (e.g. parts on a conveyor) and
overlays the running count + timestamp. Uses `CountingAlgorithm`.

![Object counting: a red counting line and running count](images/counting.png)

| Node param | Default | Description |
|---|---|---|
| `line_row` | `360` | Image row of the counting line (px from top) |
| `cluster_ths` | `5` | Minimum cluster size to count |

Put the line where objects cross, and tune the stream (ERC cap, STC) so each
object is one clean cluster — on a cluttered scene the count climbs fast (it's
meant for discrete objects). **Validated** on the Pi (bag 2026-06-16 + live
2026-06-17).

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
**black even on an obviously flickering scene** — a bright full-frame strobe
(~8 Mev/s) does exactly this. Fix: **cap `erc_rate`** in your params (e.g.
`erc_rate: 3000000`) — ERC drops on the sensor in a controlled way that preserves
periodicity, unlike the transport's random whole-packet drops — or restrict the
field of view with an `roi`.

**Validated live (Pi, 2026-06-17):** lens at f/2, camera close above a desk fan →
locked the blade-pass at ~60–70 Hz. The two things that decide success are
**optics** (focus + light) and **staying within the event-rate budget** — not the
algorithm (a synthetic 100/50 Hz input maps to exactly that frequency).

---

## Active-LED tracking — `led_tracking`

Tracks **active LED markers** — LEDs that transmit a numeric **ID** by blinking a
coded pattern — drawing a circle + decoded ID on each. The event-camera answer to
an ArUco fiducial, and the basis of active-marker motion capture (the ID solves
the *which marker is which* problem for free). Two SDK stages chained:

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
~16 mA limit) and helps the camera (see tuning). A backwards LED just won't light.

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
ros2 run rqt_image_view rqt_image_view /event_camera/led_image
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

### Tuning (validated Pi, 2026-06-17)

- **Base period is the master knob.** A word ≈ 9 blinks, so at 5 ms base it takes
  ~115 ms — the LED must stay put that long to decode, so fast motion smears the
  code and drops. Shrink the base for motion: 5 ms → 500 µs cuts the word to
  ~11 ms (~10× better). ~500 µs is the Pi's reliable floor.
- **A faster code needs a brighter blink** (the non-obvious one). Shrinking the
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
modulation conveniently **rejects steady light** (room lights, sunlight never
decode). Multi-camera extrinsic calibration is the experimental tier (needs a
second EVK4); the SDK's calibration module even supports active-LED marker boards.

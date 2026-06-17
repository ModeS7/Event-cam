# More model-free pipelines (`evk4_sdk_advanced`)

Beyond sparse [optical flow](optical_flow.md) and [object tracking](tracking.md),
the same harness drives more Metavision SDK model-free algorithms — four covered
here (dense flow, spatter, counting, frequency), plus
[active-LED tracking](led_tracking.md), which has its own page. They all
share `event_vision_node.hpp` (decode `EventPacket` → `vector<EventCD>` → an SDK
algorithm via `process_events` → publish an image), so the build, real-time
threading, and SDK-lib-path handling are identical — read
[optical_flow.md](optical_flow.md) for those. This page covers only what each
pipeline does and how to run it.

`evk4_sdk_advanced` builds every pipeline together: if you've built it once
([optical_flow.md](optical_flow.md#1-build-the-package)), these are already
built.

## One launch for all of them

**All seven pipelines** (these four, plus [optical flow](optical_flow.md),
[tracking](tracking.md), and [led_tracking](led_tracking.md)) share a single
parameterized launch — pick the pipeline with `pipeline:=`:

```bash
ros2 launch evk4_sdk_advanced pipeline.launch.py \
    pipeline:=dense_flow params_file:=$HOME/my_params.yaml
# second terminal:
ros2 run rqt_image_view rqt_image_view /event_camera/dense_flow_image
```

`pipeline` is one of `optical_flow`, `tracking`, `dense_flow`, `spatter`,
`counting`, `frequency`, `led_tracking`; each publishes
`/event_camera/<pipeline>_image`. Common launch args (`camera_name`, `serial`,
`frame_id`, `fps`, `params_file`, `debug_timing`) apply to every pipeline.
Pipeline-specific parameters (below) keep their defaults from the launch;
override them with `ros2 run … --ros-args -p name:=v`.

As always, `params_file:=$HOME/my_params.yaml` feeds your tuned driver setup
(ERC cap, biases, filters) through the driver — that, not the algorithm, is the
main latency/CPU lever on a Pi.

---

## Dense optical flow — `dense_flow`

A full **color flow field** (hue = direction, brightness = speed), vs the sparse
arrows of [optical_flow.md](optical_flow.md). Uses `TripletMatchingFlowAlgorithm`
→ `DenseFlowFrameGeneratorAlgorithm` (dense color map).

![Dense optical flow color field](images/dense_flow.png)

*Replay of the test bag: directional color along moving edges. Coverage follows
moving edges — triplet-matching flow is semi-dense, so quiet regions stay dark.*

| Parameter | Default | Description |
|---|---|---|
| `radius` | `3.0` | Spatial match search radius (px) |
| `max_flow` | `1000.0` | Matching ceiling (px/s) |
| `display_max_flow` | `300.0` | Color full-scale speed (px/s) — lower it to brighten slow scenes |

Brightness is each pixel's speed normalized to `display_max_flow`; the default
1000 px/s matching ceiling kept fast motion in range while 300 px/s lights up
ordinary motion. **Validated** on the Pi (bag replay 2026-06-16 + live by the user 2026-06-17).

## Particle / spatter tracking — `spatter`

Tracks **many small fast movers at once** (sparks, droplets, particles), each as
a small ID-labeled box over the event image. Uses `SpatterTrackerAlgorithm`.

![Spatter tracking: many small ID-labeled boxes](images/spatter.png)

*Replay of the test bag: each cluster of moving events gets a tracked box and id.*

| Parameter | Default | Description |
|---|---|---|
| `cell_size` | `7` | Tracking cell size (px); smaller = finer particles |

The node keeps the latest box per cluster id and drops ones unseen for 100 ms.
**Validated** on the Pi (bag replay 2026-06-16 + live by the user 2026-06-17).

## Object counting — `counting`

Counts objects **crossing a horizontal line** (e.g. parts on a conveyor) and
overlays the running count + timestamp on the event image. Uses
`CountingAlgorithm` + `CountingDrawingHelper`.

![Object counting: a red counting line and running count](images/counting.png)

*Replay of the test bag: the red line at row 360 with the live `Counter`. On this
busy hand scene the count climbs fast — counting is meant for discrete objects
crossing the line, not a cluttered field.*

| Parameter | Default | Description |
|---|---|---|
| `line_row` | `360` | Image row of the counting line (px from top) |
| `cluster_ths` | `5` | Minimum cluster size to count |

Put the line where objects cross it, and tune the stream (ERC cap, STC) so each
object is one clean cluster. **Validated** on the Pi (bag 2026-06-16 + live by the user 2026-06-17);
the count is meaningful only for a discrete-object scene.

## Vibration frequency — `frequency`

Estimates the **blink / vibration frequency at each pixel** and renders it as a
JET heat map with a Hz colorbar. For rotating, vibrating, or flickering scenes.
Uses `FrequencyMapAsyncAlgorithm` → `HeatMapFrameGeneratorAlgorithm`.

| Parameter | Default | Description |
|---|---|---|
| `min_freq` | `10.0` | Lowest frequency shown (Hz) — also the colorbar minimum |
| `max_freq` | `150.0` | Highest frequency shown (Hz) — also the colorbar maximum |
| `filter_length` | `7` | Estimation filter length |
| `diff_thresh_us` | `1500` | Blink-period difference threshold (µs) |

The frame is **black where no periodic motion is detected** and colors pixels by
frequency where it is — so on an ordinary (non-periodic) scene you see a black
field with the colorbar, which is correct, **not a broken node**. The SDK only
reports pixels with a sustained vibration in `[min_freq, max_freq]`, so most
everyday scenes (hand motion, walking) produce nothing.

To see it light up, point it at a genuinely periodic source: a **spinning fan**,
a **mains-powered light** (flickers at 100 Hz in 50 Hz countries / 120 Hz at
60 Hz — both in range), a vibrating tool, or a speaker cone. The signal must be
stable for ~`filter_length` (7) cycles before a pixel registers.

**Get the optics right — this matters more than anything.** A frequency map only
works on *sharp, well-lit* moving edges; an out-of-focus or dim scene produces
sensor noise (random speckle), which has no per-pixel periodicity, so nothing is
detected. If you get nothing, fix the optics before touching any parameter.

> **Recommended for reliable detection (in order of impact):**
> 1. **Open the aperture all the way (f/2)** — the most light onto the sensor.
> 2. **Light the object as brightly as you can** — point a lamp or even a **phone
>    flashlight** straight at it. More light = stronger, higher-contrast moving
>    edges = far more events to lock onto (this alone took our fan from a few
>    flickering spots to a solid detection).
> 3. **Focus precisely** — turn the focus ring until edges are crisp; check on
>    `~/image_raw` first, and if you see only uniform speckle you are out of focus.

Then **fill the frame** with the source — get close so many pixels see the
periodic signal (a small/distant fan lights up only a few pixels). A **fan** is
marginal (fast tips blur, only mid-blade pixels lock a clean period); a **slower
fan speed** and looking **straight down at the blades from close up** gives the
steadiest result.

### Keep the event rate within budget (critical for this pipeline)

Unlike the others, frequency needs **every** event at each pixel to lock a stable
period — so dropped events break it outright. If the event rate exceeds what the
node can decode+process in real time (~3 Mev/s on a Pi 5), the transport silently
drops events and the map goes **black even though a strong periodic source is
present** (no warning — it just looks like nothing is detected). A **bright,
full-frame flickering source** easily produces ~8 Mev/s and will do exactly this.

So if an obviously flickering/vibrating scene reads black, suspect the rate and
bring it down so the node keeps up:

- **Cap `erc_rate`** in your params (e.g. `erc_rate: 3000000`). ERC drops events
  *on the sensor* in a controlled way that preserves each pixel's periodicity —
  unlike the transport's random whole-packet drops, which destroy it.
- **Restrict the field of view** with an `roi` so fewer pixels are active.
- A vibrating object or a fan filling part of the frame is well within budget;
  it is mainly bright full-frame flicker that overruns it.

**Validated live on the Pi (2026-06-17):** with the lens focused at f/2 and the
camera close above a running desk fan, the map locked the **blade-pass frequency
(~60–70 Hz, amber on the bar)** over a cluster of pixels. Earlier checks
(2026-06-16) verified the algorithm + heat-map render (synthetic 100 Hz / 50 Hz →
exactly that frequency) and that a captured ~30 Hz source detects ~2200 px when
fed within budget (it drops ~79% of events at full rate). The two things that
decide success are **optics** (focus + light) and **staying within the event-rate
budget** — not the algorithm.

## Active LED / marker tracking — `led_tracking`

Tracks active LED markers that transmit a numeric ID by blinking a coded pattern
— the event-camera answer to an ArUco fiducial, and the basis of active-marker
motion capture. It has its own page covering the encoding, **building a
Raspberry-Pi LED marker** to test it, and the motion / light / reflection tuning:

**→ [led_tracking.md](led_tracking.md)**

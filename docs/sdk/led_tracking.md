# Active-LED tracking (`evk4_sdk_advanced`)

Tracks **active LED markers** — LEDs that transmit a numeric **ID** by blinking a
coded pattern — and draws a circle + the decoded ID on each, published as a ROS
image. It is the event-camera equivalent of an ArUco/AprilTag fiducial, except
the ID is sent in *time* (by the blink timing) instead of printed as a 2D
pattern. This is the building block of **active-marker motion capture**
(PhaseSpace-style): each marker announces *which* marker it is, so the camera
never has to guess correspondence.

Like the other pipelines it subscribes to the driver's events and feeds the
Metavision SDK via `process_events` — the SDK is consumed, never edited. Read
[optical_flow.md](optical_flow.md) for the build and the shared real-time harness;
this page covers what's specific to LED tracking, and **how to build a test marker
from nothing but a Raspberry Pi, an LED, and a resistor.**

## How it works

Two SDK stages chained:

```
EventCD ──▶ ModulatedLightDetectorAlgorithm ──▶ EventSourceId ──▶ ActiveLEDTrackerAlgorithm ──▶ tracks
            decodes each LED's ID from its       (per-event          one tracked circle + ID
            blink timing                          source id)          per LED
```

A plain LED — steady, or even blinking at a fixed rate — produces **no valid ID**,
so nothing is tracked; the node just streams the event image. The LED must blink
the *coded* pattern below.

## The encoding (how a marker transmits its ID)

A blink is an LED **rising edge**. The gap between consecutive rising edges, in
multiples of the **base period** `p` (= the `base_period_us` parameter), encodes a
symbol:

| Gap | Symbol |
|---|---|
| `2p` | bit `0` |
| `3p` | bit `1` |
| `4p` | start / sync |

An ID is **8 bits sent LSB-first**, framed by start symbols. Example, ID 146 =
binary `10010010` → bits LSB-first `0,1,0,0,1,0,0,1` → gaps
`2p,3p,2p,2p,3p,2p,2p,3p`, then a `4p` start to delimit the word. OFF edges and
sub-period noise are ignored by the detector.

## Build a test marker on a Raspberry Pi

You don't need a commercial marker. A GPIO-driven LED works.

### 1. Wire an LED

```
GPIO17 (header pin 11) ──[ 220–330 Ω ]──►|── GND (pin 9)
                                         LED
                                   long leg = +   short leg = –
```

- **Long leg** (anode, +) toward the resistor/GPIO; **short leg** (cathode, –,
  flat edge) to GND. A backwards LED simply won't light — swap it if unsure.
- The resistor is mandatory (the GPIO is 3.3 V). 220–330 Ω gives a safe few mA;
  **150 Ω** is brighter (~9 mA, still under the GPIO's ~16 mA limit) and helps the
  camera — see *Tuning* below.

### 2. Allow GPIO access (Pi 5 / Ubuntu)

On the Pi 5 the 40-pin header is `/dev/gpiochip4` (`pinctrl-rp1`), group
`dialout`. Add your user once:

```bash
sudo usermod -aG dialout $USER     # log out/in, or open a new session
```

### 3. Build and run the marker

The generator is [`led_marker.c`](led_marker.c):

```bash
gcc -O2 -o led_marker docs/sdk/led_marker.c
./led_marker 146 500 17           # id=146, base=500 us, GPIO17  (Ctrl+C to stop)
#            └id  └base_us └gpio_line  [└flash_us]
```

It pins itself to a CPU core and busy-waits, so the timing stays clean. To your
eye the LED looks steadily-on (it's blinking hundreds of times a second), but the
camera sees every blink. Pick any ID 0–255; different markers use different IDs so
you can track several at once.

## Run the tracker

The marker is slow (a Pi can't clock the real 200 µs reliably — see *Tuning*), so
two node parameters must match it. Put them in a small YAML:

```bash
cat > /tmp/led.yaml <<'YAML'
/**:
  ros__parameters:
    base_period_us: 500          # = the marker's base
    inactivity_period_us: 2000   # > the blink gap, so a track survives between blinks
YAML
```

Point the EVK4 at the LED (close, **focused** — a compact bright dot), then one
launch brings up the driver and the tracker:

```bash
ros2 launch evk4_sdk_advanced pipeline.launch.py pipeline:=led_tracking \
    params_file:=$HOME/my_params.yaml node_params_file:=/tmp/led.yaml
# view it:
ros2 run rqt_image_view rqt_image_view /event_camera/led_image
```

You'll see a **green circle + "146"** lock onto the LED — the decoded ID, proving
it read the exact code, not just "a blinking thing." (A real 200 µs hardware
marker needs no override — plain `pipeline:=led_tracking` with the defaults.)

| Parameter | Default | Description |
|---|---|---|
| `base_period_us` | `200` | Base blink period — **must equal the marker's base** |
| `inactivity_period_us` | `1000` | Drop a track after this long without an update; **must exceed the blink gap** |
| `num_bits` | `8` | Bits per encoded ID |
| `tolerance` | `0.1` | Allowed ±fraction on each measured gap |
| `radius` | `10.0` | Event-to-track association radius (px) |

Publishes `/event_camera/led_image` (`sensor_msgs/Image`).

## Tuning (validated on the Pi, 2026-06-17)

Validated live: GPIO17 drove an LED at ID 146; the node decoded **146** and
tracked it. Getting it *robust* taught the real trade-offs:

- **Base period is the master knob.** The ID is transmitted over one word ≈ 9
  blinks, so at base 5 ms a word takes ~115 ms — and the LED must stay roughly put
  for that long to decode. Move it fast and the code smears across pixels →
  dropout. **Shrink the base for faster motion:** 5 ms → 500 µs cuts the word to
  ~11 ms (~10× better with motion). ~500 µs is the Pi's reliable floor (below it,
  scheduler jitter corrupts more words; the start-symbol re-sync hides some).
- **A faster code needs a brighter blink (the non-obvious one).** Shrinking the
  base also shrinks the LED on-time, so each blink emits less light → fewer events
  → it falls below what the sensor needs, especially mid-motion. Add the light
  back: raise the marker's **flash** (4th arg) and/or the **LED current** (smaller
  resistor). On the Pi, 200 µs base only became usable once the flash was pushed
  up:

  ```bash
  ./led_marker 146 200 17 300     # base 200 us, but flash 300 us (vs the 50 us default)
  ```

  Flash is clamped below the smallest gap, so it never breaks the encoding (the
  detector times rising edges, not pulse length).
- **Reflections decode the same ID.** A reflection of the LED (glossy desk,
  screen, metal) blinks the identical code, so the detector reads the *same* ID
  from it — the algorithm can't tell a real LED from its mirror image. Keep
  `radius` small and **matte / angle the scene** so no shiny surface throws the
  LED back at the camera. (This is why real rigs use IR + controlled,
  non-reflective environments, sometimes polarizers.)

Rule of thumb: **small base + bright blink + small radius** tracks fast motion and
resists reflections; the Pi marker tops out around 200–500 µs. A microcontroller
(tight timing) plus a transistor-driven LED (lots of current) is what a commercial
marker uses, and what gets you past that.

## Where it goes: 3D and motion capture

One camera gives **2D position + ID** per marker. For 3D you triangulate the same
marker across **≥2 calibrated cameras** (or, for a known rigid constellation,
single-camera PnP). With IR LEDs (invisible, filterable) and several markers each
on a unique ID, that's outside-in motion capture — and the modulation conveniently
**rejects steady light** (room lights, sunlight never decode to an ID). Multi-
camera extrinsic calibration is the experimental tier in this repo (needs a second
EVK4); the Metavision SDK's own calibration module even supports active-LED marker
boards for it. The full active-*marker* tracker (`ActiveMarkerTrackerAlgorithm`,
with a marker-geometry JSON) sits a layer above this LED tracker.

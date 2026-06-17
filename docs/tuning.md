# Tuning

The default launch runs the sensor at **stock settings** — predictable, but
noisy and unbounded. This page sets up your tuned configuration once; the
result is a file, `~/my_params.yaml`, that **every later page (calibration,
rectification) launches with**.

```bash
# 1. create your config from the validated recommended one
cp $(ros2 pkg prefix evk4_bringup)/share/evk4_bringup/config/evk4_params_recommended.yaml \
   ~/my_params.yaml

# 2. launch with it -- as you will from now on
ros2 launch evk4_bringup evk4.launch.py params_file:=$HOME/my_params.yaml
```

```bash
# terminal 2 -- watch the stream while you tune (wave a hand: event cameras
# only show change)
ros2 run rqt_image_view rqt_image_view /event_camera/image_raw
```

The camera is now running the validated setup. The rest of this page
explains what that setup does — and how to tune past it.

## What your config just enabled

The recommended file ships with three things active, all validated on
hardware (EVK4 on a Raspberry Pi 5):

**ERC — the event-rate cap (`erc_mode: enabled`, `erc_rate: 10000000`).**
Event-pipeline CPU scales with the event rate, and a busy scene easily
produces more events than a small board can process — the result is a
stuttering, lagging display. The cap is enforced **on the sensor**, before
USB. 10 Mev/s (~35 MB/s) keeps rendering smooth on a Raspberry Pi 5 with
the scene still looking good (validated); raise it only for
downstream algorithms that consume the raw stream.

**Bias contrast thresholds (`bias_diff_on/off: 30`).** Biases are the
sensor's analog settings; the contrast thresholds decide how much a pixel's
brightness must change to emit an event, and they are by far the biggest
noise lever. At stock (0) a static scene crackles with salt-and-pepper
noise; 30 cuts most of it while keeping good sensitivity to faint motion —
the STC filter below cleans up the rest, which beats trading sensitivity
away with higher thresholds. Fewer noise events also means less USB,
decoding, and rendering work: on the Pi a busy scene at stock biases cost
~45% CPU with a viewer open, and raising the thresholds dropped it
substantially.

**STC trail filter (`trail_filter: true`, `stc_cut_trail` at 10000 us).**
An on-sensor filter that drops isolated events — which is exactly what
sensor noise is — so the image stays clean at the sensitive bias setting
above, and the `sharp` display mode works well (validated).
Side effect to expect: with noise gone, a static scene renders
black — and in `sharp` display mode the view can freeze entirely at a
still scene while it waits for events (correct, not broken; the default
`time_slice` keeps updating).

![Tuned event stream with the recommended settings](images/tuned_stream_demo.gif)

*The rendered stream with the recommended config (bias 30 + STC, 10 Mev/s
cap): a clean image with quiet background and crisp motion.*

## The lens: aperture and focus

The config cannot fix optics, so set these once before tuning. Aperture:
around **f/8** is the best balance — a smaller aperture deepens the depth of
field, making focus far more forgiving (closing further costs light, and
the sensor then needs more contrast to fire events). Focus: defocus is easy
to miss on an event camera because there is no static image to judge by.
Aim at something with fine, high-contrast detail (printed text works well)
at your working distance, keep events flowing by slowly moving the camera,
and turn the focus ring until the rendered edges are as thin and crisp as
possible. Moving closer and further while you turn the ring makes the
focus point much easier to find: sharpness changes fastest right around the
correct focus distance.

## The tuning loop

There are two kinds of knobs, by how a change is applied:

**Biases are live** — change them while the camera runs, per knob:

```bash
ros2 param set /event_camera bias_diff_on 60
```

or interactively with sliders:

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

**Everything else** (filters, ROI, crop, rate cap) applies when the camera
starts: uncomment the block in `~/my_params.yaml`, edit, relaunch.

Either way the loop ends the same: **the value you like goes into
`~/my_params.yaml`**, so every later launch starts there. The file is
yours, outside the repo (`git pull` never touches it), and it holds the
whole setup — biases, filters, rate cap, rendered-video settings; every
knob the sensor supports is already in it, commented out with an
explanation. (Coming from Prophesee's Metavision Studio with a tuned
`.bias` file? Copy its five values into `~/my_params.yaml` — same names,
same numbers.)

### Watch each knob work

One experiment per main parameter — run the loop and watch what each knob
does. You need a scene with some motion: point the camera at a video
playing on a screen (YouTube works fine), or just pan it around the room
by hand. Keep the viewer open, and undo each change before the next.

**Contrast thresholds — the noise floor** (live, no relaunch):

```bash
ros2 param set /event_camera bias_diff_on 0
ros2 param set /event_camera bias_diff_off 0
```

The image fills with salt-and-pepper noise — that is the sensor at stock
sensitivity. Set both back to 30 and the background goes quiet while motion
still shows.

**STC filter — the cleanup** (edit + relaunch): set `trail_filter: false`
in `~/my_params.yaml` and relaunch. The residual speckle the filter was
removing comes back. Re-enable it and relaunch.

**ERC — the event budget** (edit + relaunch): set `erc_rate: 500000` (half
a million events/s) and relaunch. Wave at the camera: motion renders thin
and ghostly because the sensor is rationing events. Set it back to
`10000000`.

**fps — display smoothness** (one-off override, no file edit):

```bash
ros2 launch evk4_bringup evk4.launch.py params_file:=$HOME/my_params.yaml fps:=5.0
```

The view turns choppy — but the raw event stream underneath is unchanged
(only the rendering changed). Relaunch without the override.

**display_type — sharp mode** (one-off override, no file edit):

```bash
ros2 launch evk4_bringup evk4.launch.py params_file:=$HOME/my_params.yaml display_type:=sharp
```

Moving edges render noticeably crisper: instead of fixed time windows,
each frame now waits for a set number of events (your STC filter keeps
that count fed with real signal). Hold the camera still at a static scene
and the view freezes until something moves — sharp trades steady timing
for sharpness. Relaunch without the override.

## The full menu

Every knob lives in your `~/my_params.yaml` as a commented, explained
block. The sensor-side groups run on the chip itself, *before* events
reach USB; the `evk4_driver` exposes **every facility the EVK4/IMX636
supports** (a facility the sensor lacks is skipped with a warning).

Biases — the sensor's analog settings, all **live** (list them with
`ros2 param list /event_camera | grep bias`):

| Bias | Range | Effect |
|---|---|---|
| `bias_diff_on` / `bias_diff_off` | -85..140 / -35..190 | Contrast threshold for ON / OFF events — higher = fewer, stronger events (less noise, less sensitivity) |
| `bias_fo` | -35..55 | Low-pass bandwidth — lower cuts high-frequency noise/flicker |
| `bias_hpf` | 0..120 | High-pass — rejects slow/global light changes |
| `bias_refr` | -20..235 | Refractory period — minimum time between events per pixel |

(Ranges are reported by the sensor itself — the driver prints them at
startup and `ros2 param describe /event_camera <bias>` shows them; values
are offsets around the factory default 0.)

The filter/region facilities — applied when the camera starts:

| Facility | Params | What it does |
|---|---|---|
| **ERC** (Event Rate Controller) | `erc_mode`, `erc_rate` | Caps total events/s on-sensor — enable when a busy scene saturates USB/CPU |
| **Trail / STC** | `trail_filter`, `trail_filter_type` (`trail`/`stc_cut_trail`/`stc_keep_trail`), `trail_filter_threshold` | Removes noise and event trails |
| **AFK** (Anti-Flicker) | `afk_enabled`, `afk_freq_low_hz`, `afk_freq_high_hz`, `afk_mode` | Rejects flickering light in a frequency band (`band_stop`) or keeps only it (`band_pass`) |
| **Digital Crop** | `digital_crop_enabled`, `digital_crop_region`, `digital_crop_reset_origin` | Drops all pixels outside one hard rectangle (vs ROI's windows) |
| **Event Mask** | `event_mask_pixels` | Blanks out individual hot pixels (limited number of hardware slots) |
| **ROI / RONI** | `roi`, `roni` | Keep (or, with `roni`, exclude) one or more `[x,y,w,h]` windows |
| **ERAF** (Event Rate Activity Filter) | `eraf_*` | *Not available on the EVK4/IMX636* — skipped with a warning; the params exist for other Prophesee sensors |

Driver-side (not on-sensor) knobs in the same file:

- `event_message_time_threshold` — message batching window. Smaller = lower
  latency, more/smaller messages.

Rendered video — these affect only the `image_raw` visualization, never
the raw event stream (`fps:=` / `display_type:=` launch arguments exist as
one-off overrides and win over the file when passed):

- `fps` (default 25.0) — how often `image_raw` frames are emitted; display
  cost scales roughly linearly with it, so raise it only with CPU to spare.
  In `sharp` mode it is a ceiling instead: frames wait for their event
  count, so quiet scenes emit below it.
- `display_type` — `time_slice` (default: all events in a fixed 40 ms
  window — steady timing at any event rate) or `sharp` (each frame waits
  for a target event COUNT — crisper edges on busy scenes, but on quiet
  scenes the count takes seconds to fill, so the view integrates seconds
  of history and feels laggy; validated on hardware). Use
  `sharp` only on busy scenes — or paired with the **STC trail filter**
  (enabled in the recommended config): with noise removed on-sensor,
  sharp's event count fills with real signal instead of speckle and the
  mode works well at ordinary scenes too (validated).

## Which knob for which symptom

**Too much noise / sparkle at a static scene.** Some salt-and-pepper events
are normal sensor shot noise. The ladder, mildest first — the bias steps
are live, so watch the image while you tune:

1. The contrast thresholds: `bias_diff_on` / `bias_diff_off` — your config
   sets the validated 30/30, with the STC filter cleaning up the rest.
   Raising them further silences more at the cost of faint-motion
   sensitivity. **This is by far the biggest lever.**
2. Lower `bias_fo` (negative values = stronger photoreceptor low-pass =
   less temporal noise). Costs a little motion sharpness.
3. Raise `bias_hpf` to reject slow-drift / ambient-flicker events.
   (On the IMX636 the effect of steps 2–3 is subtle — don't expect a visible
   change at typical noise levels; lead with step 1.)
4. The on-sensor **STC filter** (already enabled in your config) — removes
   isolated events, which is exactly what background noise is; tune
   `trail_filter_threshold` or disable it in `~/my_params.yaml` +
   relaunch. Note: with noise fully silenced, a static scene renders
   black, and in `sharp` display mode the view can freeze at a still scene
   — correct, not broken.
5. Room lighting flicker can masquerade as uniform noise. Lamps flicker at
   **twice the mains frequency** (light peaks on both AC half-cycles):
   100 Hz on 50 Hz grids (Europe), 120 Hz on 60 Hz grids (US) — enable
   `afk_enabled: true` with a band bracketing your grid's flicker
   (90–110 Hz for Europe, 110–130 Hz for the US).
6. Individual **hot pixels**: a defective pixel fires constantly regardless
   of the scene — with everything above applied, it shows as one stubborn
   dot blinking at a fixed spot in an otherwise black view (the STC filter
   can miss it, because a continuously firing pixel is not "isolated"
   noise). Find its coordinates by decoding a few seconds of events and
   counting per pixel — adapt the `event_rate` example
   ([usage.md](usage.md)): histogram the decoded `x`,`y` arrays and the hot
   pixel dominates. Then blank it on-sensor in `~/my_params.yaml`
   (flat `[x0,y0, x1,y1, ...]` list, a few hardware slots available):

   ```yaml
   event_mask_pixels: [421, 233]
   ```

**Missing faint motion** → lower `bias_diff_on`/`bias_diff_off`.

**CPU/USB saturated, frame rate collapses on a busy scene** → lowering
`erc_rate` relieves the machine, but know the trade: ERC drops events
**indiscriminately** — it cannot prefer a signal event over a noise event —
so a stream that constantly rides the cap is uniformly thinned and overall
worse. If the driver's statistics line shows the rate pinned at the cap,
retune so the scene's natural rate sits *below* it instead: raise
`bias_diff_on`/`bias_diff_off`, tune away noise (the ladder above), and add
an `roi` / `digital_crop_region` if only part of the view matters. Also
close viewers/rectification you are not using — every subscriber adds work,
and the lazy pipeline stops computing what nobody watches.

**Everything feels laggy even at a low event rate** → usually the machine
is overloaded, not the tuning — see
[troubleshooting.md](troubleshooting.md).

**Flickering lights (AC/LED) flooding events** → `afk_enabled` with a
`band_stop` bracketing the flicker (90–110 Hz in Europe); `bias_hpf` and
`erc_rate` help too.

**Blurry rendered image** → raise `fps`; on a consistently busy scene
`display_type: sharp` also helps (it lags on quiet ones — see above).

---

**Next:** [calibration.md](calibration.md) — produce a `camera_info` for
undistortion, launching with the `~/my_params.yaml` you just built.

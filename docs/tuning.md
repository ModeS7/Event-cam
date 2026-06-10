# Tuning

There are three places to tune the EVK4, by how often you change them:

| What | Where | When |
|---|---|---|
| Rendered-video fps, display mode | `evk4.launch.py` launch args | per launch |
| Driver timing, filtering, ROI | `evk4_bringup/config/evk4_params.yaml` | persistent config |
| Biases (contrast thresholds) | runtime: `ros2 param set` / `rqt_reconfigure` | live, while running |

## Rendered video (fps, display mode)

The renderer's frame rate and mode are launch arguments — they affect only
the `image_raw` visualization, not the raw event stream:

```bash
ros2 launch evk4_bringup evk4.launch.py fps:=60.0 display_type:=sharp
```

- `fps` (default 25.0) — how often `image_raw` frames are emitted.
- `display_type` — `time_slice` (all events in the frame window) or `sharp`
  (auto-tuned event count for crisper edges).

## Driver timing, filtering, ROI (`evk4_params.yaml`)

Edit `evk4_bringup/config/evk4_params.yaml` (rebuild not needed if you used
`--symlink-install`; otherwise `colcon build` to reinstall it). The common
knobs:

- `event_message_time_threshold` — message batching window. Smaller = lower
  latency, more/smaller messages.
- `roi` — restrict to a pixel rectangle to cut data volume.

### On-sensor event filters (the IMX636 ESP blocks)

These run on the sensor *before* events reach USB, so they cut data at the
source. Set them in `evk4_params.yaml`; they apply when the camera starts. Our
`evk4_driver` exposes **every facility the EVK4/IMX636 supports** (a facility
the sensor lacks is skipped with a warning):

| Facility | Params | What it does |
|---|---|---|
| **ERC** (Event Rate Controller) | `erc_mode`, `erc_rate` | Caps total events/s on-sensor — enable when a busy scene saturates USB/CPU |
| **Trail / STC** | `trail_filter`, `trail_filter_type` (`trail`/`stc_cut_trail`/`stc_keep_trail`), `trail_filter_threshold` | Removes noise and event trails |
| **AFK** (Anti-Flicker) | `afk_enabled`, `afk_freq_low_hz`, `afk_freq_high_hz`, `afk_mode` | Rejects flickering light in a frequency band (`band_stop`) or keeps only it (`band_pass`) |
| **Digital Crop** | `digital_crop_enabled`, `digital_crop_region`, `digital_crop_reset_origin` | Drops all pixels outside one hard rectangle (vs ROI's windows) |
| **Event Mask** | `event_mask_pixels` | Blanks out individual hot pixels (limited number of hardware slots) |
| **ROI / RONI** | `roi`, `roni` | Keep (or, with `roni`, exclude) one or more `[x,y,w,h]` windows |
| **ERAF** (Event Rate Activity Filter) | `eraf_*` | *Not available on the EVK4/IMX636* — skipped with a warning; the params exist for other Prophesee sensors |

## Biases — contrast thresholds (runtime)

Biases are the sensor's analog settings. The driver exposes them as **live
parameters**, so tune them while watching `image_raw` or `event_rate`:

```bash
# list current bias values
ros2 param list /event_camera | grep bias
# raise the ON-contrast threshold (fewer, stronger ON events)
ros2 param set /event_camera bias_diff_on 30
# or tune interactively
ros2 run rqt_reconfigure rqt_reconfigure
```

| Bias | Effect |
|---|---|
| `bias_diff_on` / `bias_diff_off` | Contrast threshold for ON / OFF events — higher = fewer, stronger events (less noise, less sensitivity) |
| `bias_fo` | Low-pass bandwidth — lower cuts high-frequency noise/flicker |
| `bias_hpf` | High-pass — rejects slow/global light changes |
| `bias_refr` | Refractory period — minimum time between events per pixel |

Persist a tuned set to a `.bias` file and reload it on the next launch — see
[`../evk4_bringup/config/biases/README.md`](../evk4_bringup/config/biases/README.md).

## Reducing background noise (speckle at a static scene)

Some salt-and-pepper events at a motionless scene are normal sensor shot
noise at default settings. The ladder, mildest first — the bias steps are
live (`ros2 param set`), so watch the image while you tune:

1. Raise the contrast thresholds: `bias_diff_on` / `bias_diff_off` 30–50.
   Costs sensitivity to faint motion. **This is by far the biggest lever**
   (validated on the EVK4: 50/50 silenced a static scene almost completely).
2. Lower `bias_fo` (negative values = stronger photoreceptor low-pass =
   less temporal noise). Costs a little motion sharpness.
3. Raise `bias_hpf` to reject slow-drift / ambient-flicker events.
   (On the IMX636 the effect of steps 2–3 is subtle — don't expect a visible
   change at typical noise levels; lead with step 1.)
4. Enable the on-sensor **STC filter** (`trail_filter: true`,
   `trail_filter_type: stc_cut_trail`, `trail_filter_threshold: 10000` in
   `evk4_params.yaml` + relaunch) — removes isolated events, which is
   exactly what background noise is. Note: with noise fully silenced, the
   rendered image updates only when something actually changes — a "frozen"
   viewer at a static scene is then correct, not broken.
5. Room lighting flicker (LED/fluorescent, 100/120 Hz) can masquerade as
   uniform noise — try `afk_enabled: true` (band-stop 100–120 Hz).
6. Individual hot pixels: blank them on-sensor with `event_mask_pixels`.

Fewer events also means less USB traffic, decoding, and rendering work —
on a Raspberry Pi 5 a busy scene at default biases cost ~45% CPU with the
viewer open; raising the thresholds dropped it substantially (idle pipeline
~8–16%). Tuning the sensor IS the CPU optimization.

## Which knob for which symptom

- **Too much noise / sparkle** → the noise ladder above.
- **Missing faint motion** → lower `bias_diff_on`/`bias_diff_off`.
- **CPU/USB saturated** → tune away noise events (above), enable `erc_mode` +
  `erc_rate`, or add an `roi` / `digital_crop_region`.
- **Flickering lights (AC/LED) flooding events** → enable `afk_enabled` with a
  `band_stop` around the flicker frequency (e.g. 100–120 Hz); `bias_hpf` and
  `erc_rate` help too.
- **Blurry rendered image** → raise `fps` or use `display_type:=sharp`.

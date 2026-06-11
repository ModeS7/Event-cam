# Tuning

The default launch runs the sensor at **stock settings** — predictable, but
noisy and unbounded. This page sets up your tuned configuration once; the
result is a file, `~/my_params.yaml`, that **every later page (calibration,
rectification) launches with**.

```bash
# 1. create your config from the validated recommended one (ERC rate cap;
#    see the file's comments for what each setting does and why)
cp $(ros2 pkg prefix evk4_bringup)/share/evk4_bringup/config/evk4_params_recommended.yaml \
   ~/my_params.yaml

# 2. launch with it -- as you will from now on
ros2 launch evk4_bringup evk4.launch.py params_file:=$HOME/my_params.yaml

# 3. sensor noise is already handled: the recommended config sets
#    bias_diff_on/off to 30 (balanced; 50 = very quiet). Experiment live,
#    then put the winner in ~/my_params.yaml:
ros2 param set /event_camera bias_diff_on 40
```

Edit `~/my_params.yaml` freely (it is yours, outside the repo — `git pull`
never touches it); persist the biases once you like them (workflow below)
and you never re-tune. The rest of this page explains every knob behind
that recipe.

There are three places to tune the EVK4, by how often you change them:

| What | Where | When |
|---|---|---|
| Rendered-video fps, display mode | `evk4.launch.py` launch args | per launch |
| Driver timing, filtering, ROI | `~/my_params.yaml` (driver params YAML) | persistent config |
| Biases (contrast thresholds) | `~/my_params.yaml` (startup) + `ros2 param set` (live) | persistent + live |

## Rendered video (fps, display mode)

The renderer's frame rate and mode are launch arguments — they affect only
the `image_raw` visualization, not the raw event stream:

```bash
ros2 launch evk4_bringup evk4.launch.py fps:=60.0 display_type:=sharp
```

- `fps` (default 25.0) — how often `image_raw` frames are emitted.
- `display_type` — `time_slice` (all events in the frame window) or `sharp`
  (auto-tuned event count for crisper edges).

## Driver timing, filtering, ROI (the params YAML)

These live in the driver params YAML, applied when the camera starts. The
normal place to change them is **your `~/my_params.yaml`** (created in the
recipe above): edit, relaunch with `params_file:=$HOME/my_params.yaml`,
done. (Editing the in-repo defaults at
`evk4_bringup/config/evk4_params.yaml` also works for quick experiments —
with a `--symlink-install` build it takes effect on the next launch — but
the modified file will collide with future `git pull` updates.)

The common knobs:

- `event_message_time_threshold` — message batching window. Smaller = lower
  latency, more/smaller messages.
- `roi` — restrict to a pixel rectangle to cut data volume.

### On-sensor event filters (the IMX636 ESP blocks)

These run on the sensor *before* events reach USB, so they cut data at the
source. Set them in your `~/my_params.yaml`; they apply when the camera starts. Our
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

**Tune live, persist in the YAML:** experiment with `ros2 param set`
while watching the image, then write the values you like into
`~/my_params.yaml` — they are applied at every launch, alongside everything
else (ERC, filters). One file holds the whole sensor setup. (A sensor-native
alternative, `bias_file` + the `save_biases` service, exists for
interoperability with Prophesee tooling — see
[`config/biases/README.md`](../evk4_bringup/config/biases/README.md).)

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
   `~/my_params.yaml` + relaunch) — removes isolated events, which is
   exactly what background noise is. Note: with noise fully silenced, the
   rendered image updates only when something actually changes — a "frozen"
   viewer at a static scene is then correct, not broken.
5. Room lighting flicker can masquerade as uniform noise. Lamps flicker at
   **twice the mains frequency** (light peaks on both AC half-cycles):
   100 Hz on 50 Hz grids (Europe), 120 Hz on 60 Hz grids (US) — try
   `afk_enabled: true`; the default 100–120 Hz band-stop covers both.
6. Individual hot pixels: blank them on-sensor with `event_mask_pixels`.

Fewer events also means less USB traffic, decoding, and rendering work —
on a Raspberry Pi 5 a busy scene at default biases cost ~45% CPU with the
viewer open; raising the thresholds dropped it substantially (idle pipeline
~8–16%). Tuning the sensor IS the CPU optimization.

## When the frame rate collapses on a busy scene

Event-pipeline CPU scales with the event rate. An unbounded busy scene
(sensitive biases pointed at a flickering screen reaches >10 Mev/s) can
demand more than a small board has: on a Pi 5, renderer + rectify + a viewer
saturated all four cores and the display dropped to ~12 fps even though
nothing was broken. The fix is to cap the rate **on the sensor** with ERC in
your `~/my_params.yaml`:

```yaml
erc_mode: enabled
erc_rate: 10000000     # max events/s the sensor will emit; applies at launch
```

10 Mev/s (~35 MB/s) is the validated sweet spot on a Pi 5 — smooth
rendering and the scene still looks good; at a 100 Mev/s cap (~180 MB/s
peak) the display stutters unusably. Raise beyond 10 M only for downstream
algorithms that consume the raw stream. Also close viewers/rectification
you are not using — every subscriber adds work, and the lazy pipeline stops
computing what nobody watches.

## Which knob for which symptom

- **Too much noise / sparkle** → the noise ladder above.
- **Missing faint motion** → lower `bias_diff_on`/`bias_diff_off`.
- **CPU/USB saturated** → tune away noise events (above), enable `erc_mode` +
  `erc_rate`, or add an `roi` / `digital_crop_region`.
- **Flickering lights (AC/LED) flooding events** → enable `afk_enabled` with a
  `band_stop` around the flicker frequency (e.g. 100–120 Hz); `bias_hpf` and
  `erc_rate` help too.
- **Blurry rendered image** → raise `fps` or use `display_type:=sharp`.

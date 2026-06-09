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
`--symlink-install`; otherwise `colcon build` to reinstall it). It is
commented with verified 3.0.0 defaults. The common knobs:

- `event_message_time_threshold` — message batching window. Smaller = lower
  latency, more/smaller messages.
- `erc_mode` / `erc_rate` — on-sensor Event Rate Control. Enable when the
  scene saturates the USB/CPU (see *high CPU* in troubleshooting.md).
- `trail_filter*` — on-sensor noise/trail filtering.
- `roi` — restrict to a pixel rectangle to cut data volume.

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

## Which knob for which symptom

- **Too much noise / sparkle** → raise `bias_diff_on`/`bias_diff_off`, or
  lower `bias_fo`, or enable `trail_filter`.
- **Missing faint motion** → lower `bias_diff_on`/`bias_diff_off`.
- **CPU/USB saturated** → enable `erc_mode`, set `erc_rate`, or add an `roi`.
- **Blurry rendered image** → raise `fps` or use `display_type:=sharp`.

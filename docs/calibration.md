# Calibration and rectification

The driver and renderer publish **no `CameraInfo`**, so there is no
undistortion out of the box. This repo provides a self-contained, guided
calibrator (`evk4_calibration`) plus the wiring to publish `camera_info` and
rectify with `image_proc` — no deep learning, no external toolboxes.

> **Do [tuning.md](tuning.md) first.** Calibration works on the rendered
> image, so it inherits whatever the sensor produces: at stock settings the
> stream is noisy and unbounded, which makes the pattern hard to detect and
> the preview laggy. The commands below launch with the `~/my_params.yaml`
> you created there.

> **Scope:** this calibrates and rectifies the **rendered `image_raw`**. The
> raw event stream is not an image, so event-level undistortion is a separate
> downstream step (see the end). The full loop (capture, calibrate, rectify)
> is hardware-validated on a Raspberry Pi 5 (2026-06-10).

## What you need

- A **blinking asymmetric circle grid** shown on a screen. One ships with
  this repo — open [`docs/circle_grid.html`](circle_grid.html) in any browser
  on the screen the camera will look at and press **F11** for fullscreen.
  Defaults match the calibrator (`grid_size:=5x17`, an OpenCV asymmetric
  grid drawn rotated — its bounding box is then exactly 16:9, filling a
  widescreen monitor; rotation does not affect detection). URL
  parameters: `?cols=5&rows=17&hz=2&portrait`. Set the monitor to **100%
  brightness** (avoids backlight flicker events).

  *Why circles and not a checkerboard?* A circle center is the centroid of
  hundreds of events, so it stays accurate on the speckly, soft-edged images
  an event camera produces — checkerboard corners have to be localized as
  single points on exactly those bad edges, and drift badly. Published
  event-camera calibrators reached the same conclusion
  ([E-Calib](https://github.com/mohammedsalah98/E_Calib),
  [eKalibr](https://github.com/Unsigned-Long/eKalibr)).
- A way to watch the calibrator's progress: it publishes an annotated view
  on `/calibrate/overlay`, viewed with `rqt_image_view` like every other
  image topic in this repo — on the camera machine or any other machine on
  the same ROS network (a headless camera host is fine).
- **A focused lens — check this first.** Defocus is easy to miss on an event
  camera and degrades the calibration (circle detection tolerates it better
  than checkerboard corners did, but accuracy still suffers). To focus: aim
  at the blinking grid at your working distance, watch the live image (the
  overlay, or `image_raw`), and
  turn the lens focus ring until the dots are as small and crisp as
  possible. Moving the screen (or camera) closer and further while you turn
  the ring makes the sweet spot much easier to find — sharpness changes
  fastest right around the correct focus distance.
- **Expect a choppy preview.** The rendered image only refreshes when events
  arrive — with a blinking grid that is a couple of updates per second, so
  the overlay feels laggy. That is normal (see tuning.md); move the camera
  slowly and hold each pose for a blink or two so the auto-capture can see
  it.

## 1. Run the guided calibrator

One command starts the whole session — camera, calibrator, and the progress
viewer — and shuts everything down by itself once the calibration is
written (your `~/my_params.yaml` from tuning.md carries the rate cap and
noise biases):

```bash
ros2 launch evk4_calibration calibrate.launch.py params_file:=$HOME/my_params.yaml
```

**Recommended: raise the contrast thresholds for the session.** The blinking
dots are maximum-contrast, so the sensor can be made near-silent to
everything else -- the grid then stands alone against an empty background
(values around 100 validated to give good results; from any spare terminal,
applies live):

```bash
ros2 param set /event_camera bias_diff_on 100
ros2 param set /event_camera bias_diff_off 100
```

(Or put these values in a calibration copy of your params file. They revert
to your YAML's values on the next launch.)

| Argument | Default | Description |
|---|---|---|
| `params_file` | `''` (stock) | Driver params YAML — use your `~/my_params.yaml` |
| `grid_size` | `5x17` | Circles per row x rows; must match the displayed grid |
| `output` | `event_camera.yaml` | Where the calibration is written |
| `display_type` | `time_slice` | Renderer mode (`sharp` lags on quiet scenes) |

The overlay shows the live image; when the grid is detected, colored markers
appear ON the dots (verify that — markers wandering between dots means a
detection problem). Move the camera to cover the whole field of view — near
and far, into all four image corners, and tilted at angles. The four bars
(X / Y / Size / Skew) fill green as coverage improves; the tool
**auto-captures** good views, and once coverage is complete it **calibrates
by itself**: it logs the RMS reprojection error in the launch terminal
(lower is better; under ~0.5 px is good), writes `event_camera.yaml` in the
directory it was started from, and the whole session shuts down. Ctrl+C
aborts without writing anything.

## 2. Use the calibration: camera_info and rectification

The YAML from step 1 does nothing by itself. This step wires it into the
running system: the launch publishes it as `/event_camera/camera_info` (the
standard message that geometry tools read, stamped to match each `image_raw`
frame) and runs `image_proc` rectification, which uses it to undistort the
image — `/event_camera/image_rect`. Viewing the rectified stream is also how
you judge the calibration's quality.

One-time install (`image_proc` is not part of `ros-desktop`):

```bash
sudo apt install ros-$ROS_DISTRO-image-proc
```

```bash
# terminal 1 -- camera + camera_info + rectification, with your tuned config
ros2 launch evk4_bringup evk4.launch.py \
    calibration_url:=$(pwd)/event_camera.yaml rectify:=true \
    params_file:=$HOME/my_params.yaml
```

```bash
# terminal 2 -- view the undistorted image
ros2 run rqt_image_view rqt_image_view /event_camera/image_rect
```

To judge the calibration, hold something straight (door frame, monitor edge)
near the image **corners**: straight lines in `image_rect` = good; curved or
smeared corners = recalibrate with better corner coverage.

(Omit `rectify:=true` if you only need `camera_info` published for a
downstream tool; rectification runs in the camera container and costs
nothing while nobody subscribes to `image_rect`.)

To keep a calibration with the repo, copy the YAML into
`evk4_bringup/config/calibration/` (it ships a zero-distortion placeholder).

## Frames and TF

`camera_info` copies `image_raw`'s `frame_id`, so the two always share a frame
and `image_proc` pairs them cleanly — regardless of what the driver stamped.

The driver stamps every message with the `frame_id` launch argument
(default `event_camera_optical_frame`). Verify with:

```bash
ros2 topic echo /event_camera/events --once    # look at header.frame_id
```

To place the camera in a robot TF tree, publish a static transform from your
base frame to the camera frame (substitute your mount offsets):

```bash
ros2 run tf2_ros static_transform_publisher \
    --x 0.10 --y 0 --z 0.05 --yaw 0 --pitch 0 --roll 0 \
    --frame-id base_link --child-frame-id event_camera_optical_frame
```

## Event-level undistortion (downstream)

`image_proc` rectifies the *rendered* image only. To undistort the raw events
themselves, remap each event's `(x, y)` with the same intrinsics in your own
consumer — e.g. precompute an undistortion lookup from `K` and `D`
(`cv2.initUndistortRectifyMap` / `cv2.undistortPoints`) and apply it per event.
This repo does not do it for you; the calibration file is the input you'd use.

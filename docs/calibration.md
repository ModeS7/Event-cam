# Calibration and rectification

The driver and renderer publish **no `CameraInfo`**, so there is no
undistortion out of the box. This repo provides a self-contained, guided
calibrator (`evk4_calibration`) plus the wiring to publish `camera_info` and
rectify with `image_proc` — no deep learning, no external toolboxes.

> **Do [tuning.md](tuning.md) first.** Calibration works on the rendered
> image, so it inherits whatever the sensor produces: at stock settings the
> stream is noisy and unbounded, which makes the board hard to detect and
> the preview laggy. The commands below use the recommended tuned config
> from that page.

> **Scope:** this calibrates and rectifies the **rendered `image_raw`**. The
> raw event stream is not an image, so event-level undistortion is a separate
> downstream step (see the end). The full loop (capture, calibrate, rectify)
> is hardware-validated on a Raspberry Pi 5 (2026-06-10).

## What you need

- A **checkerboard** of known geometry. A **flickering** checkerboard shown on
  a screen works best: an event camera only sees change, so flashing makes the
  whole board appear. One ships with this repo — open
  [`docs/checkerboard.html`](checkerboard.html) in any browser on the screen
  the camera will look at and press **F11** for fullscreen. Defaults match the
  calibrator (9x7 squares = `board_size:=8x6`, blinking at 2 Hz); change them
  with URL parameters, e.g. `checkerboard.html?cols=10&rows=8&hz=4`. Set the
  monitor to **100% brightness** (avoids backlight flicker events). A printed
  board moved briskly also works.
- A machine with a **display** for the calibration window (run on a desktop,
  or over X-forwarding / VNC if the camera is on a headless Pi).
- **A focused lens — check this first.** Defocus is easy to miss on an event
  camera and ruins corner detection (edges render thick and soft, board
  corners look rounded). To focus: aim at the flickering board at your
  working distance, watch `image_raw` live, and turn the lens focus ring
  until the square edges are as thin and crisp as possible. Moving the
  board (or camera) closer and further while you turn the ring makes the
  sweet spot much easier to find — sharpness changes fastest right around
  the correct focus distance.
- **Expect a choppy preview.** The rendered image only refreshes when events
  arrive — with a blinking board that is a couple of updates per second, so
  the calibration window feels laggy. That is normal (see tuning.md); move
  the camera slowly and hold each pose for a blink or two so the
  auto-capture can see it.

## 1. Run the guided calibrator

```bash
# terminal 1 -- camera + renderer, with the tuned config (rate cap);
# 'sharp' display mode is cleanest to detect on
ros2 launch evk4_bringup evk4.launch.py display_type:=sharp \
    params_file:=$(ros2 pkg prefix evk4_bringup)/share/evk4_bringup/config/evk4_params_recommended.yaml
```

```bash
# any spare terminal -- cut sensor noise so the board stands out
ros2 param set /event_camera bias_diff_on 30
ros2 param set /event_camera bias_diff_off 30
```

```bash
# terminal 2 -- the calibrator
# board_size = inner corners (cols x rows); square_size = square edge in metres
ros2 run evk4_calibration calibrate --ros-args \
    -p board_size:=8x6 -p square_size:=0.025 \
    -r image_raw:=/event_camera/image_raw
```

A window opens showing the live image and the detected board. Move the board
to cover the whole field of view — near and far, all corners, and tilted at
angles. The four bars (X / Y / Size / Skew) fill green as coverage improves;
the tool **auto-captures** good views, so just keep moving until it says
`READY`. Then press **`c`** to calibrate.

Keys: **SPACE** force-capture the current view, **`c`** calibrate, **`q`** quit.

It prints the RMS reprojection error (lower is better; under ~0.5 px is good)
and writes `event_camera.yaml` in the current directory.

## 2. Publish camera_info and rectify

Point `calibration_url` at the file you just made; the `camera_info_publisher`
echoes it as `CameraInfo` stamped to match each `image_raw` frame. Each
long-running command below gets its own terminal (see usage.md):

```bash
# terminal 1 -- the camera, with the calibration wired in
ros2 launch evk4_bringup evk4.launch.py \
    calibration_url:=$(pwd)/event_camera.yaml
# -> /event_camera/camera_info
```

Rectification uses `image_proc`, which is **not** part of `ros-desktop` —
install it first (verified needed on a fresh Pi 5 install, 2026-06-09):

```bash
sudo apt install ros-$ROS_DISTRO-image-proc
```

The simplest way to rectify is to let the launch compose it into the camera
container (`rectify:=true`) — the image then reaches `image_proc` as a
pointer instead of being serialized between processes, which matters at
high frame rates on small boards:

```bash
# terminal 1 -- camera + camera_info + rectification in one container
# (keep the tuned params_file from step 1 here too)
ros2 launch evk4_bringup evk4.launch.py \
    calibration_url:=$(pwd)/event_camera.yaml rectify:=true \
    params_file:=$(ros2 pkg prefix evk4_bringup)/share/evk4_bringup/config/evk4_params_recommended.yaml
```

```bash
# terminal 2 -- view the undistorted image
ros2 run rqt_image_view rqt_image_view /event_camera/image_rect
```

(`image_proc rectify_node` can also be run as a standalone node remapped to
the same topics; the composed form is just cheaper.)

To judge the calibration, hold something straight (door frame, monitor edge)
near the image **corners**: straight lines in `image_rect` = good; curved or
smeared corners = recalibrate with better corner coverage.

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

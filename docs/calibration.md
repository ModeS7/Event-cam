# Calibration and rectification

The driver and renderer publish **no `CameraInfo`**, so there is no
undistortion out of the box. This repo provides a self-contained, guided
calibrator (`evk4_calibration`) plus the wiring to publish `camera_info` and
rectify with `image_proc` — no deep learning, no external toolboxes.

> **Scope:** this calibrates and rectifies the **rendered `image_raw`**. The
> raw event stream is not an image, so event-level undistortion is a separate
> downstream step (see the end). Not yet validated on hardware.

## What you need

- A **checkerboard** of known geometry. A **flickering** checkerboard shown on
  a screen works best: an event camera only sees change, so flashing makes the
  whole board appear. A printed board moved briskly also works.
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
# terminal 1 -- camera + renderer; 'sharp' mode is cleanest to detect on
ros2 launch evk4_bringup evk4.launch.py display_type:=sharp
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

Then rectify:

```bash
# terminal 2 -- rectification (prints NOTHING when healthy; that is normal)
ros2 run image_proc rectify_node --ros-args \
    -r image:=/event_camera/image_raw \
    -r camera_info:=/event_camera/camera_info \
    -r image_rect:=/event_camera/image_rect
```

```bash
# terminal 3 -- view the undistorted image
ros2 run rqt_image_view rqt_image_view /event_camera/image_rect
```

To judge the calibration, hold something straight (door frame, monitor edge)
near the image **corners**: straight lines in `image_rect` = good; curved or
smeared corners = recalibrate with better corner coverage.

To keep a calibration with the repo, copy the YAML into
`evk4_bringup/config/calibration/` (it ships a zero-distortion placeholder).

## Frames and TF

`camera_info` copies `image_raw`'s `frame_id`, so the two always share a frame
and `image_proc` pairs them cleanly — regardless of what the driver stamped.

Note the `frame_id` launch arg is passed to the driver but **driver 3.0.0
ignores it and stamps the last 4 digits of the serial number** (e.g. `1701`);
newer drivers honor it. Check the actual frame with:

```bash
ros2 topic echo /event_camera/events --once    # look at header.frame_id
```

To place the camera in a robot TF tree, publish a static transform from your
base frame to that camera frame (substitute your mount offsets and the frame
the messages actually carry):

```bash
ros2 run tf2_ros static_transform_publisher \
    --x 0.10 --y 0 --z 0.05 --yaw 0 --pitch 0 --roll 0 \
    --frame-id base_link --child-frame-id 1701
```

## Event-level undistortion (downstream)

`image_proc` rectifies the *rendered* image only. To undistort the raw events
themselves, remap each event's `(x, y)` with the same intrinsics in your own
consumer — e.g. precompute an undistortion lookup from `K` and `D`
(`cv2.initUndistortRectifyMap` / `cv2.undistortPoints`) and apply it per event.
This repo does not do it for you; the calibration file is the input you'd use.

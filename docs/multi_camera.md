# Multiple cameras

The bringup is parameterized per camera, so running two or more EVK4s is just
launching it once per camera with a distinct `camera_name` and `serial`. Each
gets its own namespace (`/cam0/events`, `/cam1/events`, ...) and its own
container, biases, and calibration.

> The wiring below follows from the per-camera bringup. Extrinsic (stereo)
> calibration is covered as a future extension at the end of this page.

## Hardware

- **One USB 3.x connection per camera.** A single EVK4 already pushes tens of
  MB/s; two on the same USB controller can saturate it. Prefer separate USB3
  host ports, and use Event Rate Control (`erc_*` in your params YAML — see tuning.md) if you
  saturate.
- **A sync cable** between cameras if you need time-aligned streams (you do,
  for 3D). One camera drives the sync signal (`primary`), the others follow it
  (`secondary`).

## Run two cameras

Find each serial from the driver's startup log (or launch one at a time). Then,
in separate terminals:

```bash
# primary: generates the sync signal
ros2 launch evk4_bringup evk4.launch.py \
    camera_name:=cam0 serial:=00050591 sync_mode:=primary

# secondary: follows the primary's sync
ros2 launch evk4_bringup evk4.launch.py \
    camera_name:=cam1 serial:=00051701 sync_mode:=secondary
```

Add `params_file:=$HOME/my_params.yaml` to each launch to keep your tuned
setup ([tuning.md](tuning.md)), as on every other page. You now have
`/cam0/events`, `/cam1/events`, `/cam0/image_raw`,
`/cam1/image_raw`, each in its own container. Add a third camera with another
`secondary` launch. More than two `secondary` cameras is fine.

## Synchronization

Without hardware sync the two event streams drift, and any 3D reconstruction
that assumes simultaneous observations will be wrong. `sync_mode` sets the
driver's role; the **physical sync cable** must connect the primary's sync
output to each secondary's sync input — see the Prophesee EVK4 manual for the
connector and signal details.

`primary` and `secondary` are only for an actual synced rig: a lone `primary`
coordinates with sync hardware and may not stream on its own. For a
**single camera, use the default `standalone`.**

## Per-camera intrinsic calibration

Calibrate each camera separately (see [calibration.md](calibration.md)), giving
each its own output file and `calibration_url`. The calibrator **starts no
camera** — it only subscribes to `image_raw`, so the camera must already be
running. The one-command `calibrate.launch.py` assumes the default camera name, so
with multiple cameras run the calibrator node directly and remap it per camera.

For cam0, in three terminals:

```bash
# terminal 1 — bring up cam0 with your tuned setup
ros2 launch evk4_bringup evk4.launch.py \
    camera_name:=cam0 serial:=00050591 params_file:=$HOME/my_params.yaml

# terminal 2 — calibrate against cam0's image, writing cam0.yaml
ros2 run evk4_calibration calibrate --ros-args \
    -p output:=cam0.yaml -r image_raw:=/cam0/image_raw

# terminal 3 — watch the calibrator's annotated view
ros2 run rqt_image_view rqt_image_view /calibrate/overlay
```

Repeat for cam1 (`output:=cam1.yaml`, `-r image_raw:=/cam1/image_raw`). Then bring
each camera up with its own intrinsics, still passing your tuned setup:

```bash
ros2 launch evk4_bringup evk4.launch.py camera_name:=cam0 serial:=00050591 \
    sync_mode:=primary params_file:=$HOME/my_params.yaml \
    calibration_url:=$(pwd)/cam0.yaml
```

## Frames, TF, and extrinsic calibration (future work)

Each camera publishes in its own `frame_id`. For 3D you also need the
**relative poses between cameras** (extrinsics) and a TF tree linking them to a
common frame.

This repo does **not** yet provide extrinsic/stereo calibration. The pieces it
would build on are already here: per-camera intrinsics, per-camera frames, and
hardware sync. The intended approach when a multi-camera rig exists:

1. Hardware-sync the cameras (above) so views are simultaneous.
2. Capture synchronized views of one shared blinking circle grid across all
   cameras.
3. Run stereo/multi-camera calibration (`cv2.stereoCalibrate` for a pair, or a
   bundle adjustment for more) to recover each camera's pose.
4. Publish those poses as static transforms so the cameras share a TF tree.

A future `evk4_calibration` stereo mode is the natural home for steps 2–3.
Until there is real multi-camera hardware to develop against, this is left as a
documented extension point rather than guessed-at code.

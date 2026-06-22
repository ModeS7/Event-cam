# Documentation

Setup is a **four-page sequence** — each page builds on the previous one and hands
its artifact to the next. Read in order:

1. [installation.md](installation.md) — apt packages, the udev rule, build, smoke
   test → live events on screen.
2. [usage.md](usage.md) — terminals, launch arguments, topics and QoS, viewing,
   consuming events, recording.
3. [tuning.md](tuning.md) — noise, the event-rate cap, filters → your
   `~/my_params.yaml`, which every later page launches with.
4. [calibration.md](calibration.md) — guided intrinsic calibration →
   `event_camera.yaml` for camera_info / rectification.

Reference, as needed:

- [troubleshooting.md](troubleshooting.md) — camera not found, permissions, no
  events, poisoned communication.
- [multi_camera.md](multi_camera.md) — namespaces, hardware sync, per-camera
  calibration.

Advanced (optional):

- [sdk/](sdk/README.md) — the Metavision SDK Pro layer: model-free, cv3d, and
  ML/GPU pipelines on the event stream. To add your own, see
  [sdk/extending.md](sdk/extending.md).

New here? Start with the [project README](../README.md) for the quickstart, the
platform table, and the topic contract.

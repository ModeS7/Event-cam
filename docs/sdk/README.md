# Metavision SDK Pro — the advanced layer (optional)

This directory documents an **opt-in** layer that runs **closed-source Metavision
SDK** computer-vision algorithms on the EVK4 event stream. The base repo
(driver, renderer, examples, calibration) is built on **OpenEB** and needs none
of this — read on only if you want SDK algorithms like optical flow, tracking,
or detection.

It is deliberately self-contained: the package here (`evk4_sdk_advanced`)
**subscribes to `/event_camera/events`** — the same topic everything else uses —
decodes it, and feeds the events to the SDK through its camera-independent
`process_events` API. So the SDK touches the repo in exactly one optional
package, behind the normal event stream. If you never install the SDK, nothing
else changes; a normal `colcon build` simply skips this package.

## The pages

**Setup (once):**
1. [access.md](access.md) — get a Prophesee account and an identity token (the
   gated credential; free for EVK4 owners).
2. [install.md](install.md) — install the SDK: `apt` on x86_64, or a source
   build on ARM (Raspberry Pi / Jetson).

**Pipelines** (build together; same launch shape and real-time harness):
3. [optical_flow.md](optical_flow.md) — **sparse optical flow**: event edges
   with flow-vector arrows.
4. [tracking.md](tracking.md) — **object tracking**: labeled bounding boxes on
   moving objects.

## What's implemented vs. planned

`evk4_sdk_advanced` ships **two** pipelines — sparse optical flow and object
tracking — sharing one real-time harness (`event_vision_node.hpp`: decode
`EventPacket` → `vector<EventCD>` → an SDK algorithm → publish an image). Adding
a pipeline is three small hooks plus a launch. The SDK's `cv` and `analytics`
modules hold many more model-free algorithms (dense flow, counting,
frequency/vibration, particle tracking) that slot into the same harness; ML
detection and stereo calibration are the heavier, experimental extensions.

## Validation matrix

Honest evidence levels — **validated** means we ran it on hardware and recorded
the result; **expected** means inferred from an adjacent result, not run;
**not viable** means structurally blocked.

| Capability | x86_64 | Raspberry Pi 5 (ARM64) | Jetson (ARM64) |
|---|---|---|---|
| SDK build | expected (apt binaries) | **validated** (source, 2026-06-16) | expected (source) |
| Sparse optical flow | expected | **validated** (2026-06-16) | expected (from Pi) |
| Object tracking | expected | **validated** (2026-06-16) | expected (from Pi) |
| Other CV / analytics (model-free) | expected | expected | expected |
| ML detection (needs LibTorch) | expected (CPU/GPU) | **not viable** (no CUDA) | expected (CUDA, untested) |
| Stereo calibration | untested skeleton | untested skeleton | untested skeleton |

This project validates on a **Raspberry Pi 5** (the constrained, worst-case
target) and an x86 dev machine; Jetson is covered only by inference (same
aarch64 + Ubuntu) and is **not** tested here. Never read "expected" as "works."

## Platform note

The SDK ships **prebuilt apt binaries for x86_64 only**. On ARM (Pi, Jetson) you
**build it from source** — a validated, ~22-minute procedure on a Pi 5
([install.md](install.md)). Everything downstream (`evk4_sdk_advanced`, the
launch, the results) is identical once the SDK is present.

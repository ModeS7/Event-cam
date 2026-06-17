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
5. [more_pipelines.md](more_pipelines.md) — five more model-free pipelines on the
   same harness: **dense flow**, **particle/spatter tracking**, **counting**,
   **vibration frequency**, and **active-LED tracking**.

## What's implemented vs. planned

`evk4_sdk_advanced` ships **seven** pipelines — sparse optical flow, object
tracking, dense flow, particle/spatter tracking, counting, vibration frequency,
and active-LED tracking — all sharing one real-time harness
(`event_vision_node.hpp`: decode `EventPacket` → `vector<EventCD>` → an SDK
algorithm → publish an image). Adding a pipeline is three small hooks plus a
launch entry. ML detection and stereo calibration are the heavier, experimental
extensions still ahead (they need LibTorch / two cameras).

## Validation matrix

Honest evidence levels — **validated** means we ran it on hardware and recorded
the result; **expected** means inferred from an adjacent result, not run;
**not viable** means structurally blocked.

| Capability | x86_64 | Raspberry Pi 5 (ARM64) | Jetson (ARM64) |
|---|---|---|---|
| SDK build | expected (apt binaries) | **validated** (source, 2026-06-16) | expected (source) |
| Sparse optical flow | expected | **validated** (2026-06-16) | expected (from Pi) |
| Object tracking | expected | **validated** (2026-06-16) | expected (from Pi) |
| Dense flow / spatter / counting | expected | **validated** (2026-06-16, bag) | expected (from Pi) |
| Frequency (vibration) | expected | **validated** (2026-06-17, fan ~60-70 Hz) | expected (from Pi) |
| Active-LED tracking | expected | **validated** (2026-06-17, Pi-GPIO LED marker, ID 146) | expected (from Pi) |
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

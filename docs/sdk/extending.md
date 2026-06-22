# Adding a pipeline

A developer guide for wrapping a new Metavision SDK algorithm as an
`evk4_sdk_advanced` pipeline — the same pattern every shipped pipeline follows.
Use it to add one of the gated apps (ArUco, model-3D, …) or any other SDK
algorithm yourself. Assumes some C++ and ROS 2 familiarity; for running the
existing pipelines, see [pipelines.md](pipelines.md).

## The contract

- **One SDK algorithm per ROS node.** The node subscribes to
  `/event_camera/events`, decodes EVT3 to `vector<Metavision::EventCD>`, and feeds
  the events to the SDK through its camera-independent `process_events` API.
- **The SDK is consumed, never modified.** You link the SDK libraries and call
  them; you do not edit the SDK.
- **Library APIs only — do not vendor SDK *sample* code.** This repo is Apache-2.0
  and must not contain proprietary Metavision files. Anything in the SDK's
  `cpp_samples` / `samples` tree is sample source, not a library API. The clearest
  case is **ArUco**: the SDK's marker detection lives in a bundled `aruco_nano.h`
  sample, not the `cv3d` library, so it cannot be shipped here — reimplement it
  with OpenCV's own `cv::aruco` instead. If the capability you want is a real
  library class (in a module `.so`, e.g. `Model3dTrackingAlgorithm` in `cv3d`),
  it is fair game.

## The harness in brief

Every pipeline subclasses one of two bases in `include/evk4_sdk_advanced/`:

- **`EventVisionNode`** (`event_vision_node.hpp`) — for model-free / cheap-per-packet
  algorithms. **Two threads:** the *subscription thread* decodes each packet and
  runs your algorithm incrementally; a *frame thread*, paced to `fps`, renders one
  newest frame on demand and publishes it. A mutex guards only a cheap result swap,
  so rendering never stalls ingestion.
- **`MlVisionNode`** (`ml_vision_node.hpp`, extends `EventVisionNode`) — for GPU
  inference. Adds a **third, dedicated inference thread** so a ~50 ms model never
  blocks ingestion (see [Rules & gotchas](#rules--gotchas)).

`EventVisionNode` calls five hooks you implement:

| Hook | Thread | Lock | Job |
|---|---|---|---|
| `onInit(w, h)` | subscription (once) | no | create the SDK algorithm(s) + frame generator now the sensor size is known |
| `processEvents(events)` | subscription | no | run the algorithm on this packet; stash results in a member |
| `stageResults()` | subscription | **holds `mutex()`** | copy those results into a staging member, atomically with the frame's events + timestamp |
| `swapResults()` | frame | **holds `mutex()`** | swap the staged results into frame-thread-local buffers |
| `renderFrame(events, ts, frame)` | frame | no | draw the BGR image; return `false` to skip publishing this tick |

**Member-ownership rule** — keep every member in exactly one of three zones, and
the locked sections cheap (never compute under `mutex()`):

- *subscription-thread* members (the algorithm, scratch buffers),
- *staged* members (written by `stageResults`, read by `swapResults`, both under `mutex()`),
- *frame-thread* members (the frame generator, the working result copy).

Publishing is **lazy**: the base does nothing while no one is subscribed.

## Worked example — `tracking` end to end

`src/tracking_node.cpp` wraps `Metavision::TrackingAlgorithm` into ID-labeled
boxes. Condensed:

```cpp
#include "evk4_sdk_advanced/event_vision_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <metavision/sdk/analytics/algorithms/tracking_algorithm.h>
#include <metavision/sdk/analytics/utils/tracking_drawing.h>
#include <metavision/sdk/core/algorithms/on_demand_frame_generation_algorithm.h>

namespace evk4_sdk_advanced
{
class Tracking : public EventVisionNode
{
public:
  explicit Tracking(const rclcpp::NodeOptions & options)
  : EventVisionNode("tracking", "tracking_image", options)   // node name, image topic
  {
    min_size_ = static_cast<int>(declare_parameter("min_size", 10));
    max_size_ = static_cast<int>(declare_parameter("max_size", 300));
  }
  ~Tracking() override { stopFrameThread(); }   // join the frame thread before members die

protected:
  void onInit(uint16_t w, uint16_t h) override
  {
    tracker_ = std::make_unique<Metavision::TrackingAlgorithm>(w, h, Metavision::TrackingConfig{});
    frame_gen_ = std::make_unique<Metavision::OnDemandFrameGenerationAlgorithm>(w, h, accTimeUs());
  }

  // subscription thread, no lock: run the algorithm on this packet
  void processEvents(const std::vector<Metavision::EventCD> & ev) override
  {
    tr_out_.clear();
    tracker_->process_events(ev.begin(), ev.end(), std::back_inserter(tr_out_));
  }

  // under mutex(): copy results into staging, atomically with the frame's events+stamp
  void stageResults() override { /* keep the latest box per object id in latest_ */ }
  // frame thread, under mutex(): hand staged results to the frame thread
  void swapResults() override  { /* latest_ -> work_tr_ */ }

  // frame thread, no lock: draw the event image, then the boxes
  bool renderFrame(
    const std::vector<Metavision::EventCD> & ev, Metavision::timestamp ts, cv::Mat & frame) override
  {
    if (ev.empty()) { return false; }
    frame_gen_->process_events(ev.begin(), ev.end());
    frame_gen_->generate(ts, frame);
    Metavision::draw_tracking_results(ts, work_tr_.cbegin(), work_tr_.cend(), frame);
    return true;
  }

private:
  int min_size_{10}, max_size_{300};
  std::unique_ptr<Metavision::TrackingAlgorithm> tracker_;                  // sub thread
  std::vector<Metavision::EventTrackingData> tr_out_;                       // sub thread
  std::map<std::size_t, Metavision::EventTrackingData> latest_;             // staged (mutex())
  std::vector<Metavision::EventTrackingData> work_tr_;                      // frame thread
  std::unique_ptr<Metavision::OnDemandFrameGenerationAlgorithm> frame_gen_; // frame thread
};
}  // namespace evk4_sdk_advanced

RCLCPP_COMPONENTS_REGISTER_NODE(evk4_sdk_advanced::Tracking)
```

Three things wire it in:

**1. The node** — constructor passes `("<name>", "<name>_image", options)`; the last
line registers the component. Use node parameters (`declare_parameter`) for tunables.

**2. `CMakeLists.txt`** — one `add_sdk_component(...)` line, naming the source, the
fully-qualified class (the plugin), the executable name, and the `MetavisionSDK::*`
modules to link:

```cmake
add_sdk_component(tracking_component src/tracking_node.cpp
  "evk4_sdk_advanced::Tracking" tracking
  MetavisionSDK::base MetavisionSDK::core MetavisionSDK::analytics)
```

Put it in the **model-free block** (after the `find_package(MetavisionSDK ...
COMPONENTS base core cv analytics)` near the top). `add_sdk_component` builds the
shared library + standalone executable, links OpenCV, sets the RPATH so the SDK
`.so`s resolve at runtime, and installs everything.

**3. The launch** — nothing to write. `pipeline.launch.py` is generic: it runs the
executable named by `pipeline:=`, remaps `events` and `<name>_image` under the
camera namespace. Just add `<name>` to the launch's help text. The topic must be
`<name>_image` for the remap to match.

Build and run:

```bash
colcon build --packages-select evk4_sdk_advanced --cmake-args \
  -DMetavisionSDK_DIR=$HOME/metavision_src/openeb-5.3.1/build/generated/share/cmake/MetavisionSDKCMakePackagesFilesDir
source install/setup.bash
ros2 launch evk4_sdk_advanced pipeline.launch.py pipeline:=tracking params_file:=$HOME/my_params.yaml
ros2 run rqt_image_view rqt_image_view /event_camera/tracking_image      # second terminal
```

## The cv3d tier delta

`cv3d` algorithms (edgelet, model-3D, …) build only when the SDK is configured
with `-DUSE_SOPHUS=ON` ([install.md](install.md)). The CMake gates the components
behind a `find_package` so a build without it just skips them:

```cmake
find_package(MetavisionSDK QUIET COMPONENTS cv3d)
if(TARGET MetavisionSDK::cv3d)
  add_sdk_component(edgelet_component src/edgelet_node.cpp
    "evk4_sdk_advanced::EdgeletTracking" edgelet
    MetavisionSDK::base MetavisionSDK::core MetavisionSDK::cv MetavisionSDK::cv3d)
endif()
```

The node (`src/edgelet_node.cpp`) is an ordinary `EventVisionNode` — the only
difference from `tracking` is which SDK headers/classes it uses. Add
`-DSophus_DIR=/opt/ros/$ROS_DISTRO/share/sophus/cmake` to the `colcon` cmake-args
so the package resolves `cv3d`'s transitive Sophus dependency.

## The ML tier delta

GPU inference pipelines subclass **`MlVisionNode`** instead, which loads a
TorchScript/ONNX model, builds the SDK `EventPreprocessor`, slices the stream into
`delta_t_us` windows, and runs the model on its own thread. You implement three
ML hooks instead of `processEvents`/`renderFrame`:

| Hook | Thread | Job |
|---|---|---|
| `onModelReady(json, w, h)` | subscription (once) | set up output-specific state from the model JSON (labels, NMS, tracker) |
| `extractResults(ts)` | **inference** (under `mutex()`) | read `modelOutput()` into members after each `infer()` |
| `drawResults(frame)` | frame | overlay results on the event frame |

`src/gesture_node.cpp` is the minimal example (softmax-argmax of the class logits;
`MlVisionNode("gesture", "gesture_image", options)`). The CMake gates it behind
Torch + the SDK `ml` module:

```cmake
find_package(Torch QUIET)
find_package(MetavisionSDK QUIET COMPONENTS ml)
if(Torch_FOUND AND TARGET MetavisionSDK::ml)
  add_sdk_component(gesture_component src/gesture_node.cpp
    "evk4_sdk_advanced::GestureClassification" gesture
    MetavisionSDK::base MetavisionSDK::core MetavisionSDK::ml ${TORCH_LIBRARIES})
endif()
```

ML pipelines take `model_path`, `gpu_id`, and `delta_t_us` as node params (pass via
`node_params_file:=`). Build with `-DTorch_DIR=...` and `nvcc` on `PATH`.

## Rules & gotchas

- **Keep heavy work off the ingestion thread.** `processEvents` runs on the
  subscription thread; if it costs more than real-time, ingestion backs up, packets
  drop, and the renderer starves. Model-free algorithms are fine per-packet. A
  ~50 ms GPU inference is **not** — that is exactly why `MlVisionNode` runs the
  model on a separate thread (it once dropped detection to ~0 fps inline).
- **Destructor ordering.** The frame thread (and, for ML, the inference thread)
  call your virtuals, so they must be **joined before your members are destroyed**.
  A model-free derived destructor calls `stopFrameThread()`; an `MlVisionNode`
  derived destructor calls `stopThreads()`. Both are idempotent; forgetting one is
  a use-after-free on shutdown.
- **Never compute under `mutex()`.** `stageResults`/`swapResults` should be cheap
  copies/swaps only; the heavy render happens in `renderFrame`, outside the lock.
- **`accumulation_time_us`** (default 10 ms) is the event window the frame
  generator draws; expose it or override per pipeline if your visual needs more.
- **Lazy by default** — the base skips all decode/compute while nothing subscribes;
  you get that for free.

## New-pipeline checklist

1. `src/<name>_node.cpp` — subclass `EventVisionNode` (or `MlVisionNode`),
   constructor `("<name>", "<name>_image", options)`, implement the hooks,
   destructor calls `stopFrameThread()` (or `stopThreads()`), end with
   `RCLCPP_COMPONENTS_REGISTER_NODE`.
2. `CMakeLists.txt` — one `add_sdk_component(<name>_component src/<name>_node.cpp
   "evk4_sdk_advanced::<Class>" <name> MetavisionSDK::base MetavisionSDK::core <modules>)`
   in the right tier block.
3. `launch/pipeline.launch.py` — add `<name>` to the help text (no logic change).
4. Build with the appropriate cmake-args (`-DMetavisionSDK_DIR=`, plus `-DSophus_DIR=`
   for cv3d or `-DTorch_DIR=` for ML).
5. Run `pipeline:=<name>`, view `/event_camera/<name>_image`, and validate with
   `debug_timing:=true` (frame/s + Mev/s) or a bag replay.

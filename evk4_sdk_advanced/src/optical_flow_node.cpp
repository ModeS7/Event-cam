// Sparse optical flow on the EVK4 event stream: event edges with flow-vector
// arrows overlaid, published as a ROS image. A subclass of EventVisionNode,
// which provides the real-time decode + threading harness (see
// event_vision_node.hpp). The SDK is consumed via process_events, never edited.

#include "evk4_sdk_advanced/event_vision_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <metavision/sdk/core/algorithms/on_demand_frame_generation_algorithm.h>
#include <metavision/sdk/cv/algorithms/sparse_flow_frame_generator_algorithm.h>
#include <metavision/sdk/cv/algorithms/sparse_optical_flow_algorithm.h>
#include <metavision/sdk/cv/events/event_optical_flow.h>

#include <algorithm>
#include <memory>
#include <mutex>
#include <vector>

namespace evk4_sdk_advanced
{

class OpticalFlow : public EventVisionNode
{
public:
  explicit OpticalFlow(const rclcpp::NodeOptions & options)
  : EventVisionNode("optical_flow", "flow_image", options)
  {
    RCLCPP_INFO(get_logger(), "sparse optical flow: %.0f fps", fps());
  }

  ~OpticalFlow() override { stopFrameThread(); }  // join before our members die

protected:
  void onInit(uint16_t width, uint16_t height) override
  {
    flow_algo_ = std::make_unique<Metavision::SparseOpticalFlowAlgorithm>(
      width, height, Metavision::SparseOpticalFlowConfig::Preset::FastObjects);
    flow_frame_gen_ = std::make_unique<Metavision::SparseFlowFrameGeneratorAlgorithm>();
    frame_gen_ = std::make_unique<Metavision::OnDemandFrameGenerationAlgorithm>(
      width, height, accTimeUs());
    RCLCPP_INFO(get_logger(), "pipeline initialized for %ux%u", width, height);
  }

  // Sub thread: flow incrementally on this packet (no lock).
  void processEvents(const std::vector<Metavision::EventCD> & events) override
  {
    flow_out_.clear();
    flow_algo_->process_events(events.begin(), events.end(), std::back_inserter(flow_out_));
  }

  // Stage the flow events (base holds mutex(), atomic with the frame timestamp).
  void stageResults() override
  {
    staging_fl_.insert(staging_fl_.end(), flow_out_.begin(), flow_out_.end());
  }

  void swapResults() override { std::swap(staging_fl_, work_fl_); }

  // Frame thread: render the event image, overlay this window's flow arrows.
  bool renderFrame(
    const std::vector<Metavision::EventCD> & events, Metavision::timestamp ts,
    cv::Mat & frame) override
  {
    if (events.empty()) {
      return false;
    }
    frame_gen_->process_events(events.begin(), events.end());
    frame_gen_->generate(ts, frame);
    flow_buf_.insert(flow_buf_.end(), work_fl_.begin(), work_fl_.end());
    overlayFlow(ts, frame);
    return true;
  }

private:
  void overlayFlow(Metavision::timestamp ts, cv::Mat & frame)
  {
    if (flow_buf_.empty()) {
      return;
    }
    const Metavision::timestamp ts_begin = ts - static_cast<Metavision::timestamp>(accTimeUs());
    auto it_begin = std::lower_bound(
      flow_buf_.begin(), flow_buf_.end(), ts_begin,
      [](const Metavision::EventOpticalFlow & ev, Metavision::timestamp t) { return ev.t < t; });
    auto it_end = std::upper_bound(
      flow_buf_.begin(), flow_buf_.end(), ts,
      [](Metavision::timestamp t, const Metavision::EventOpticalFlow & ev) { return t < ev.t; });
    if (it_begin != it_end) {
      flow_frame_gen_->add_flow_for_frame_update(it_begin, it_end);
    }
    flow_frame_gen_->update_frame_with_flow(frame);
    flow_frame_gen_->clear_ids();
    const Metavision::timestamp ts_remove =
      ts - static_cast<Metavision::timestamp>(accTimeUs()) +
      static_cast<Metavision::timestamp>(1e6 / fps());
    auto it_remove = std::lower_bound(
      flow_buf_.begin(), flow_buf_.end(), ts_remove,
      [](const Metavision::EventOpticalFlow & ev, Metavision::timestamp t) { return ev.t < t; });
    flow_buf_.erase(flow_buf_.begin(), it_remove);
  }

  std::unique_ptr<Metavision::SparseOpticalFlowAlgorithm> flow_algo_;  // sub thread
  std::vector<Metavision::EventOpticalFlow> flow_out_;                 // sub thread
  std::vector<Metavision::EventOpticalFlow> staging_fl_;               // shared (mutex())
  std::unique_ptr<Metavision::SparseFlowFrameGeneratorAlgorithm> flow_frame_gen_;  // frame thread
  std::unique_ptr<Metavision::OnDemandFrameGenerationAlgorithm> frame_gen_;        // frame thread
  std::vector<Metavision::EventOpticalFlow> work_fl_;                  // frame thread
  std::vector<Metavision::EventOpticalFlow> flow_buf_;                 // frame thread
};

}  // namespace evk4_sdk_advanced

RCLCPP_COMPONENTS_REGISTER_NODE(evk4_sdk_advanced::OpticalFlow)

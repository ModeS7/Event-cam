// Dense optical flow on the EVK4 event stream: a full color flow field (hue =
// direction, value = speed), vs the sparse arrows of optical_flow. Subclass of
// EventVisionNode; the SDK is consumed via process_events.

#include "evk4_sdk_advanced/event_vision_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <metavision/sdk/cv/algorithms/dense_flow_frame_generator_algorithm.h>
#include <metavision/sdk/cv/algorithms/triplet_matching_flow_algorithm.h>
#include <metavision/sdk/cv/configs/triplet_matching_flow_algorithm_config.h>
#include <metavision/sdk/cv/events/event_optical_flow.h>

#include <iterator>
#include <memory>
#include <mutex>
#include <vector>

namespace evk4_sdk_advanced
{

class DenseFlow : public EventVisionNode
{
public:
  explicit DenseFlow(const rclcpp::NodeOptions & options)
  : EventVisionNode("dense_flow", "dense_flow_image", options)
  {
    radius_ = declare_parameter("radius", 3.0);
    max_flow_ = declare_parameter("max_flow", 1000.0);          // matching ceiling, px/s
    display_max_flow_ = declare_parameter("display_max_flow", 300.0);  // color-map ceiling, px/s
    RCLCPP_INFO(
      get_logger(), "dense optical flow: %.0f fps, match<=%.0f px/s, color full-scale %.0f px/s",
      fps(), max_flow_, display_max_flow_);
  }

  ~DenseFlow() override { stopFrameThread(); }

protected:
  void onInit(uint16_t width, uint16_t height) override
  {
    const Metavision::TripletMatchingFlowAlgorithmConfig config(
      static_cast<float>(radius_), 10.0f, static_cast<float>(max_flow_));
    flow_algo_ = std::make_unique<Metavision::TripletMatchingFlowAlgorithm>(width, height, config);
    flow_gen_ = std::make_unique<Metavision::DenseFlowFrameGeneratorAlgorithm>(
      width, height, static_cast<float>(display_max_flow_), 1.0f,
      Metavision::DenseFlowFrameGeneratorAlgorithm::VisualizationMethod::DenseColorMap);
    RCLCPP_INFO(get_logger(), "pipeline initialized for %ux%u", width, height);
  }

  void processEvents(const std::vector<Metavision::EventCD> & events) override
  {
    flow_out_.clear();
    flow_algo_->process_events(events.begin(), events.end(), std::back_inserter(flow_out_));
  }

  void stageResults() override
  {
    staging_fl_.insert(staging_fl_.end(), flow_out_.begin(), flow_out_.end());
  }

  // Clear first so the swap leaves staging_fl_ empty for the next interval --
  // a plain swap would put the previous frame's flow back into staging, where
  // stageResults keeps appending, so work_fl_ would grow without bound and the
  // dense field would stack every frame on top of the last.
  void swapResults() override
  {
    work_fl_.clear();
    std::swap(staging_fl_, work_fl_);
  }

  // renderFrame draws only the dense color field, never the events -> skip staging.
  bool rendersEvents() const override { return false; }

  // Render the dense color flow field (no event image: DenseColorMap fills the
  // whole frame).
  bool renderFrame(
    const std::vector<Metavision::EventCD> &, Metavision::timestamp, cv::Mat & frame) override
  {
    if (work_fl_.empty()) {
      return false;
    }
    flow_gen_->process_events(work_fl_.begin(), work_fl_.end());
    flow_gen_->generate(frame);
    flow_gen_->reset();
    return true;
  }

private:
  double radius_{3.0};
  double max_flow_{1000.0};
  double display_max_flow_{300.0};
  std::unique_ptr<Metavision::TripletMatchingFlowAlgorithm> flow_algo_;  // sub thread
  std::vector<Metavision::EventOpticalFlow> flow_out_;                   // sub thread
  std::vector<Metavision::EventOpticalFlow> staging_fl_;                 // shared (mutex())
  std::vector<Metavision::EventOpticalFlow> work_fl_;                    // frame thread
  std::unique_ptr<Metavision::DenseFlowFrameGeneratorAlgorithm> flow_gen_;  // frame thread
};

}  // namespace evk4_sdk_advanced

RCLCPP_COMPONENTS_REGISTER_NODE(evk4_sdk_advanced::DenseFlow)

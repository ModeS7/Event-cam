// Object counting on the EVK4 event stream: counts objects crossing one or more
// horizontal lines (e.g. parts on a conveyor) and overlays the running count on
// the event image. Subclass of EventVisionNode; the SDK is consumed via
// process_events with a synchronous output callback.

#include "evk4_sdk_advanced/event_vision_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <metavision/sdk/analytics/algorithms/counting_algorithm.h>
#include <metavision/sdk/analytics/utils/mono_counting_status.h>
#include <metavision/sdk/analytics/utils/counting_drawing_helper.h>
#include <metavision/sdk/core/algorithms/on_demand_frame_generation_algorithm.h>

#include <memory>
#include <vector>

namespace evk4_sdk_advanced
{

class Counting : public EventVisionNode
{
public:
  explicit Counting(const rclcpp::NodeOptions & options)
  : EventVisionNode("counting", "counting_image", options)
  {
    cluster_ths_ = static_cast<int>(declare_parameter("cluster_ths", 5));
    line_row_ = static_cast<int>(declare_parameter("line_row", 360));
    RCLCPP_INFO(
      get_logger(), "object counting: %.0f fps, line at row %d, cluster_ths %d",
      fps(), line_row_, cluster_ths_);
  }

  ~Counting() override { stopFrameThread(); }

protected:
  void onInit(uint16_t width, uint16_t height) override
  {
    count_algo_ = std::make_unique<Metavision::CountingAlgorithm>(
      width, height, cluster_ths_, accTimeUs());
    count_algo_->add_line_counter(line_row_);
    // The callback fires synchronously inside process_events (sub thread); store
    // the latest count for staging.
    count_algo_->set_output_callback(
      [this](const Metavision::CountingAlgorithm::OutputEvent & e) {
        count_ = e.second.global_counter;
      });
    draw_helper_ = std::make_unique<Metavision::CountingDrawingHelper>(
      std::vector<int>{line_row_});
    frame_gen_ = std::make_unique<Metavision::OnDemandFrameGenerationAlgorithm>(
      width, height, accTimeUs());
    RCLCPP_INFO(get_logger(), "pipeline initialized for %ux%u", width, height);
  }

  void processEvents(const std::vector<Metavision::EventCD> & events) override
  {
    if (!events.empty()) {
      count_algo_->process_events(events.begin(), events.end());
    }
  }

  void stageResults() override { staged_count_ = count_; }

  void swapResults() override { work_count_ = staged_count_; }

  bool renderFrame(
    const std::vector<Metavision::EventCD> & events, Metavision::timestamp ts,
    cv::Mat & frame) override
  {
    if (events.empty()) {
      return false;
    }
    frame_gen_->process_events(events.begin(), events.end());
    frame_gen_->generate(ts, frame);
    draw_helper_->draw(ts, work_count_, frame);
    return true;
  }

private:
  int cluster_ths_{5};
  int line_row_{360};
  int count_{0};           // sub thread (set in callback)
  int staged_count_{0};    // shared (mutex())
  int work_count_{0};      // frame thread
  std::unique_ptr<Metavision::CountingAlgorithm> count_algo_;             // sub thread
  std::unique_ptr<Metavision::CountingDrawingHelper> draw_helper_;        // frame thread
  std::unique_ptr<Metavision::OnDemandFrameGenerationAlgorithm> frame_gen_;  // frame thread
};

}  // namespace evk4_sdk_advanced

RCLCPP_COMPONENTS_REGISTER_NODE(evk4_sdk_advanced::Counting)

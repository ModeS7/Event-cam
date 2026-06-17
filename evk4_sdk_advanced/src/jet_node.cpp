// Jet monitoring on the EVK4 event stream: detects + counts jets (dispensed dots)
// by spotting event-rate peaks inside a detection ROI, overlaying the ROI, the
// running jet count, and the ROI event rate on the event image. For monitoring
// dispensing processes. Subclass of EventVisionNode; SDK via process_events.

#include "evk4_sdk_advanced/event_vision_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <metavision/sdk/analytics/algorithms/jet_monitoring_algorithm.h>
#include <metavision/sdk/analytics/configs/jet_monitoring_algorithm_config.h>
#include <metavision/sdk/analytics/utils/jet_monitoring_drawing_helper.h>
#include <metavision/sdk/core/algorithms/on_demand_frame_generation_algorithm.h>

#include <opencv2/core.hpp>

#include <algorithm>
#include <memory>
#include <vector>

namespace evk4_sdk_advanced
{

class JetMonitoring : public EventVisionNode
{
public:
  explicit JetMonitoring(const rclcpp::NodeOptions & options)
  : EventVisionNode("jet_monitoring", "jet_monitoring_image", options)
  {
    roi_x_ = static_cast<int>(declare_parameter("roi_x", 600));
    roi_y_ = static_cast<int>(declare_parameter("roi_y", 330));
    roi_w_ = static_cast<int>(declare_parameter("roi_w", 80));
    roi_h_ = static_cast<int>(declare_parameter("roi_h", 60));
    th_up_kevps_ = static_cast<int>(declare_parameter("th_up_kevps", 50));
    th_down_kevps_ = static_cast<int>(declare_parameter("th_down_kevps", 10));
    jet_accumulation_us_ = static_cast<int>(declare_parameter("jet_accumulation_us", 500));
    time_step_us_ = static_cast<int>(declare_parameter("time_step_us", 50));
    RCLCPP_INFO(
      get_logger(), "jet monitoring: %.0f fps, ROI [%d,%d,%d,%d], th_up %d kev/s",
      fps(), roi_x_, roi_y_, roi_w_, roi_h_, th_up_kevps_);
  }

  ~JetMonitoring() override { stopFrameThread(); }

protected:
  void onInit(uint16_t width, uint16_t height) override
  {
    roi_ = cv::Rect(roi_x_, roi_y_, roi_w_, roi_h_);
    Metavision::JetMonitoringAlgorithmConfig config;
    config.detection_roi = roi_;
    config.time_step_us = time_step_us_;
    config.accumulation_time_us = jet_accumulation_us_;
    config.th_up_kevps = th_up_kevps_;
    config.th_down_kevps = th_down_kevps_;
    config.th_up_delay_us = 100;
    config.th_down_delay_us = 0;
    jet_ = std::make_unique<Metavision::JetMonitoringAlgorithm>(config);
    // Fires synchronously inside process_events (sub thread) when a jet is detected.
    jet_->set_on_jet_callback([this](const Metavision::EventJet &) { ++count_; });
    draw_ = std::make_unique<Metavision::JetMonitoringDrawingHelper>(
      cv::Rect(0, 0, width, height), roi_, config.nozzle_orientation);
    frame_gen_ = std::make_unique<Metavision::OnDemandFrameGenerationAlgorithm>(
      width, height, accTimeUs());
    RCLCPP_INFO(get_logger(), "pipeline initialized for %ux%u", width, height);
  }

  void processEvents(const std::vector<Metavision::EventCD> & events) override
  {
    if (!events.empty()) {
      jet_->process_events(events.begin(), events.end());
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
    // ROI event rate (kev/s) for the on-image readout.
    long in_roi = 0;
    for (const auto & e : events) {
      if (roi_.contains(cv::Point(e.x, e.y))) {
        ++in_roi;
      }
    }
    const int er_kevps = static_cast<int>(in_roi * 1000 / std::max<uint32_t>(1, accTimeUs()));
    draw_->draw(ts, work_count_, er_kevps, frame);
    return true;
  }

private:
  int roi_x_{600}, roi_y_{330}, roi_w_{80}, roi_h_{60};
  int th_up_kevps_{50}, th_down_kevps_{10}, jet_accumulation_us_{500}, time_step_us_{50};
  cv::Rect roi_;
  int count_{0};        // sub thread (jet callback)
  int staged_count_{0}, work_count_{0};
  std::unique_ptr<Metavision::JetMonitoringAlgorithm> jet_;                   // sub thread
  std::unique_ptr<Metavision::JetMonitoringDrawingHelper> draw_;             // frame thread
  std::unique_ptr<Metavision::OnDemandFrameGenerationAlgorithm> frame_gen_;  // frame thread
};

}  // namespace evk4_sdk_advanced

RCLCPP_COMPONENTS_REGISTER_NODE(evk4_sdk_advanced::JetMonitoring)

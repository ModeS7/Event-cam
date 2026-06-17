// Active LED / marker tracking on the EVK4 event stream: detects modulated-light
// sources (LEDs that transmit an ID by frequency modulation) and tracks them,
// overlaying a circle + id per LED on the event image. Two-stage SDK pipeline:
//   EventCD -> ModulatedLightDetectorAlgorithm -> EventSourceId
//           -> ActiveLEDTrackerAlgorithm       -> EventActiveTrack
// Produces tracks ONLY when modulated active-LED markers are in view (encoded
// with num_bits / base_period_us); otherwise it just streams the event image.
// Subclass of EventVisionNode; the SDK is consumed via process_events, with the
// current track set delivered by the tracker's monitoring callback.

#include "evk4_sdk_advanced/event_vision_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <metavision/sdk/cv/algorithms/active_led_tracker_algorithm.h>
#include <metavision/sdk/cv/algorithms/modulated_light_detector_algorithm.h>
#include <metavision/sdk/cv/events/event_active_track.h>
#include <metavision/sdk/cv/events/event_source_id.h>
#include <metavision/sdk/core/algorithms/on_demand_frame_generation_algorithm.h>

#include <opencv2/imgproc.hpp>

#include <iterator>
#include <memory>
#include <vector>

namespace evk4_sdk_advanced
{

class LedTracking : public EventVisionNode
{
public:
  explicit LedTracking(const rclcpp::NodeOptions & options)
  : EventVisionNode("led_tracking", "led_tracking_image", options)
  {
    radius_ = declare_parameter("radius", 10.0);
    inactivity_us_ = static_cast<int>(declare_parameter("inactivity_period_us", 1000));
    num_bits_ = static_cast<int>(declare_parameter("num_bits", 8));
    base_period_us_ = static_cast<int>(declare_parameter("base_period_us", 200));
    tolerance_ = declare_parameter("tolerance", 0.1);
    RCLCPP_INFO(
      get_logger(), "active LED tracking: %.0f fps, radius %.0f px, %d-bit @ %d us base period",
      fps(), radius_, num_bits_, base_period_us_);
  }

  ~LedTracking() override { stopFrameThread(); }

protected:
  void onInit(uint16_t width, uint16_t height) override
  {
    Metavision::ModulatedLightDetectorAlgorithm::Params detector_params;
    detector_params.width = width;
    detector_params.height = height;
    detector_params.num_bits = static_cast<std::uint8_t>(num_bits_);
    detector_params.base_period_us = static_cast<std::uint32_t>(base_period_us_);
    detector_params.tolerance = static_cast<float>(tolerance_);
    detector_ = std::make_unique<Metavision::ModulatedLightDetectorAlgorithm>(detector_params);

    Metavision::ActiveLEDTrackerAlgorithm::Params params;
    params.radius = static_cast<float>(radius_);
    params.inactivity_period_us = static_cast<Metavision::timestamp>(inactivity_us_);
    params.monitoring_frequency_hz = static_cast<float>(fps());
    // The monitoring callback fires inside process_events (sub thread) at the
    // monitoring frequency, delivering the current set of active tracks.
    led_algo_ = std::make_unique<Metavision::ActiveLEDTrackerAlgorithm>(
      params,
      [](std::uint32_t) { return true; },
      [this](const std::vector<Metavision::ActiveLEDTrackerAlgorithm::ActiveTrack> & tracks) {
        monitored_ = tracks;
      });
    frame_gen_ = std::make_unique<Metavision::OnDemandFrameGenerationAlgorithm>(
      width, height, accTimeUs());
    RCLCPP_INFO(get_logger(), "pipeline initialized for %ux%u", width, height);
  }

  void processEvents(const std::vector<Metavision::EventCD> & events) override
  {
    if (events.empty()) {
      return;
    }
    // Stage 1: decode modulated-light source IDs (one EventSourceId per EventCD).
    source_ids_.clear();
    detector_->process_events(events.begin(), events.end(), std::back_inserter(source_ids_));
    // Stage 2: track the identified sources; the monitoring callback fills monitored_.
    sink_.clear();  // per-event track output is unused; we read the monitoring set
    led_algo_->process_events(source_ids_.begin(), source_ids_.end(), std::back_inserter(sink_));
  }

  void stageResults() override { staged_ = monitored_; }

  void swapResults() override { std::swap(staged_, work_); }

  bool renderFrame(
    const std::vector<Metavision::EventCD> & events, Metavision::timestamp ts,
    cv::Mat & frame) override
  {
    if (events.empty()) {
      return false;
    }
    frame_gen_->process_events(events.begin(), events.end());
    frame_gen_->generate(ts, frame);
    for (const auto & t : work_) {
      const cv::Point c(static_cast<int>(t.x), static_cast<int>(t.y));
      cv::circle(frame, c, static_cast<int>(radius_), cv::Scalar(0, 255, 0), 2);
      cv::putText(
        frame, std::to_string(t.id), c + cv::Point(8, -8),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }
    return true;
  }

private:
  double radius_{10.0};
  int inactivity_us_{1000};
  int num_bits_{8};
  int base_period_us_{200};
  double tolerance_{0.1};
  std::unique_ptr<Metavision::ModulatedLightDetectorAlgorithm> detector_;   // sub thread
  std::vector<Metavision::EventSourceId> source_ids_;                       // sub thread
  std::vector<Metavision::EventActiveTrack> sink_;                          // sub thread
  std::vector<Metavision::ActiveLEDTrackerAlgorithm::ActiveTrack> monitored_;  // sub thread (callback)
  std::vector<Metavision::ActiveLEDTrackerAlgorithm::ActiveTrack> staged_;     // shared (mutex())
  std::vector<Metavision::ActiveLEDTrackerAlgorithm::ActiveTrack> work_;       // frame thread
  std::unique_ptr<Metavision::ActiveLEDTrackerAlgorithm> led_algo_;          // sub thread
  std::unique_ptr<Metavision::OnDemandFrameGenerationAlgorithm> frame_gen_;  // frame thread
};

}  // namespace evk4_sdk_advanced

RCLCPP_COMPONENTS_REGISTER_NODE(evk4_sdk_advanced::LedTracking)

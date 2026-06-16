// Vibration frequency estimation on the EVK4 event stream: estimates the blink /
// vibration frequency at each pixel and renders it as a JET heat map with a
// colorbar (Hz). Best on periodically moving / flickering scenes. Subclass of
// EventVisionNode; the SDK is consumed via process_events with an async output
// callback.

#include "evk4_sdk_advanced/event_vision_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <metavision/sdk/analytics/algorithms/frequency_map_async_algorithm.h>
#include <metavision/sdk/analytics/algorithms/heat_map_frame_generator_algorithm.h>
#include <metavision/sdk/cv/configs/frequency_estimation_config.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <memory>
#include <string>
#include <vector>

namespace evk4_sdk_advanced
{

class Frequency : public EventVisionNode
{
public:
  explicit Frequency(const rclcpp::NodeOptions & options)
  : EventVisionNode("frequency", "frequency_image", options)
  {
    min_freq_ = declare_parameter("min_freq", 10.0);    // Hz
    max_freq_ = declare_parameter("max_freq", 150.0);   // Hz
    filter_length_ = static_cast<int>(declare_parameter("filter_length", 7));
    diff_thresh_us_ = static_cast<int>(declare_parameter("diff_thresh_us", 1500));
    RCLCPP_INFO(
      get_logger(), "frequency map: %.0f fps, range %.0f-%.0f Hz",
      fps(), min_freq_, max_freq_);
  }

  ~Frequency() override { stopFrameThread(); }

protected:
  void onInit(uint16_t width, uint16_t height) override
  {
    const Metavision::FrequencyEstimationConfig config(
      static_cast<unsigned int>(filter_length_),
      static_cast<float>(min_freq_), static_cast<float>(max_freq_),
      static_cast<unsigned int>(diff_thresh_us_), false);
    freq_algo_ = std::make_unique<Metavision::FrequencyMapAsyncAlgorithm>(width, height, config);
    freq_algo_->set_update_frequency(static_cast<float>(fps()));
    // Async callback: fires inside process_events (sub thread) ONLY when the
    // algorithm actually detects vibration -- on a non-periodic scene it never
    // fires, which is why the map stays the seeded black below. Just keep the
    // latest map; the frame thread reports the detection count (renderFrame).
    freq_algo_->set_output_callback(
      [this](Metavision::timestamp, Metavision::FrequencyMapAsyncAlgorithm::OutputMap & m) {
        latest_map_ = m.clone();
      });
    heat_gen_ = std::make_unique<Metavision::HeatMapFrameGeneratorAlgorithm>(
      static_cast<float>(min_freq_), static_cast<float>(max_freq_), 1.0f, width, height, "Hz");
    // Seed with an empty (all-zero -> below min_freq -> black) map so the node
    // always publishes a frame with the colorbar, even on non-periodic scenes;
    // it stays black until vibrating/flickering objects appear.
    latest_map_ = cv::Mat1f::zeros(height, width);
    RCLCPP_INFO(get_logger(), "pipeline initialized for %ux%u", width, height);
  }

  void processEvents(const std::vector<Metavision::EventCD> & events) override
  {
    if (!events.empty()) {
      freq_algo_->process_events(events.begin(), events.end());
    }
  }

  // renderFrame draws only the heat map, never the events -> skip render staging.
  bool rendersEvents() const override { return false; }

  // Frequency needs every event for per-pixel periodicity: dropped events break
  // detection outright, so warn loudly when the node can't keep up.
  bool warnOnOverload() const override { return true; }

  void stageResults() override
  {
    if (!latest_map_.empty()) {
      staged_map_ = latest_map_;  // shallow ref-count copy; cloned in the callback
    }
  }

  void swapResults() override { std::swap(staged_map_, work_map_); }

  // Render the heat map (full frame + colorbar; no event image).
  bool renderFrame(
    const std::vector<Metavision::EventCD> &, Metavision::timestamp, cv::Mat & frame) override
  {
    if (work_map_.empty()) {
      return false;
    }
    heat_gen_->generate_bgr_heat_map(work_map_, frame);
    // Status line, so a black frame is never mistaken for a broken node: the
    // heat map only colors pixels that register a 10-150 Hz vibration, so a
    // non-periodic scene is correctly all-black.
    const int detected = cv::countNonZero(work_map_);
    const cv::Scalar color = detected > 0 ? cv::Scalar(0, 255, 0) : cv::Scalar(80, 80, 80);
    const std::string status = detected > 0
      ? std::to_string(detected) + " px vibrating"
      : "no vibration (point at a fan / flickering light)";
    cv::putText(frame, status, cv::Point(8, 24), cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 1);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "vibration: %d pixels in [%.0f, %.0f] Hz", detected, min_freq_, max_freq_);
    return true;
  }

private:
  double min_freq_{10.0};
  double max_freq_{150.0};
  int filter_length_{7};
  int diff_thresh_us_{1500};
  cv::Mat1f latest_map_;   // sub thread (set in callback)
  cv::Mat1f staged_map_;   // shared (mutex())
  cv::Mat1f work_map_;     // frame thread
  std::unique_ptr<Metavision::FrequencyMapAsyncAlgorithm> freq_algo_;        // sub thread
  std::unique_ptr<Metavision::HeatMapFrameGeneratorAlgorithm> heat_gen_;     // frame thread
};

}  // namespace evk4_sdk_advanced

RCLCPP_COMPONENTS_REGISTER_NODE(evk4_sdk_advanced::Frequency)

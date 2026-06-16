// Object tracking on the EVK4 event stream: bounding boxes on moving objects,
// overlaid on the event image. A subclass of EventVisionNode (the real-time
// decode + threading harness); the SDK is consumed via process_events.

#include "evk4_sdk_advanced/event_vision_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <metavision/sdk/analytics/algorithms/tracking_algorithm.h>
#include <metavision/sdk/analytics/configs/tracking_algorithm_config.h>
#include <metavision/sdk/analytics/events/event_tracking_data.h>
#include <metavision/sdk/analytics/utils/tracking_drawing.h>
#include <metavision/sdk/core/algorithms/on_demand_frame_generation_algorithm.h>

#include <cstdint>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

namespace evk4_sdk_advanced
{

class Tracking : public EventVisionNode
{
public:
  explicit Tracking(const rclcpp::NodeOptions & options)
  : EventVisionNode("tracking", "tracking_image", options)
  {
    min_size_ = static_cast<int>(declare_parameter("min_size", 10));
    max_size_ = static_cast<int>(declare_parameter("max_size", 300));
    RCLCPP_INFO(
      get_logger(), "object tracking: %.0f fps, object size %d..%d px",
      fps(), min_size_, max_size_);
  }

  ~Tracking() override { stopFrameThread(); }  // join before our members die

protected:
  void onInit(uint16_t width, uint16_t height) override
  {
    Metavision::TrackingConfig config;  // sensible defaults
    tracker_ = std::make_unique<Metavision::TrackingAlgorithm>(
      static_cast<int>(width), static_cast<int>(height), config);
    tracker_->set_min_size(static_cast<std::uint16_t>(min_size_));
    tracker_->set_max_size(static_cast<std::uint16_t>(max_size_));
    frame_gen_ = std::make_unique<Metavision::OnDemandFrameGenerationAlgorithm>(
      width, height, accTimeUs());
    RCLCPP_INFO(get_logger(), "pipeline initialized for %ux%u", width, height);
  }

  // Sub thread (no lock): run the tracker on this packet.
  void processEvents(const std::vector<Metavision::EventCD> & events) override
  {
    tr_out_.clear();
    tracker_->process_events(events.begin(), events.end(), std::back_inserter(tr_out_));
  }

  // base holds mutex(): the tracker emits many updates per object over a frame;
  // keep only the LATEST box per object id and drop ones not seen recently,
  // otherwise the frame fills with stacked boxes.
  void stageResults() override
  {
    if (tr_out_.empty()) {
      return;
    }
    const Metavision::timestamp now = tr_out_.back().t;
    for (const auto & o : tr_out_) {
      latest_[o.object_id_] = o;
    }
    for (auto it = latest_.begin(); it != latest_.end();) {
      if (now - it->second.t > kPruneUs) {
        it = latest_.erase(it);
      } else {
        ++it;
      }
    }
  }

  void swapResults() override
  {
    work_tr_.clear();
    for (const auto & kv : latest_) {
      work_tr_.push_back(kv.second);
    }
  }

  bool renderFrame(
    const std::vector<Metavision::EventCD> & events, Metavision::timestamp ts,
    cv::Mat & frame) override
  {
    if (events.empty()) {
      return false;
    }
    frame_gen_->process_events(events.begin(), events.end());
    frame_gen_->generate(ts, frame);
    if (!work_tr_.empty()) {
      Metavision::draw_tracking_results(ts, work_tr_.cbegin(), work_tr_.cend(), frame);
    }
    return true;
  }

private:
  static constexpr Metavision::timestamp kPruneUs = 100000;  // drop objects unseen 100 ms

  int min_size_{10};
  int max_size_{300};
  std::unique_ptr<Metavision::TrackingAlgorithm> tracker_;            // sub thread
  std::vector<Metavision::EventTrackingData> tr_out_;                 // sub thread
  std::map<std::size_t, Metavision::EventTrackingData> latest_;       // shared (mutex())
  std::vector<Metavision::EventTrackingData> work_tr_;                // frame thread
  std::unique_ptr<Metavision::OnDemandFrameGenerationAlgorithm> frame_gen_;  // frame thread
};

}  // namespace evk4_sdk_advanced

RCLCPP_COMPONENTS_REGISTER_NODE(evk4_sdk_advanced::Tracking)

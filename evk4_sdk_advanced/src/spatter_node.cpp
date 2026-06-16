// Spatter / particle tracking on the EVK4 event stream: tracks many small fast
// movers and overlays a box per cluster on the event image. Subclass of
// EventVisionNode; the SDK is consumed via process_events.

#include "evk4_sdk_advanced/event_vision_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <metavision/sdk/analytics/algorithms/spatter_tracker_algorithm.h>
#include <metavision/sdk/analytics/configs/spatter_tracker_algorithm_config.h>
#include <metavision/sdk/analytics/events/event_spatter_cluster.h>
#include <metavision/sdk/analytics/utils/tracking_drawing.h>
#include <metavision/sdk/core/algorithms/on_demand_frame_generation_algorithm.h>

#include <iterator>
#include <map>
#include <memory>
#include <vector>

namespace evk4_sdk_advanced
{

class SpatterTracking : public EventVisionNode
{
public:
  explicit SpatterTracking(const rclcpp::NodeOptions & options)
  : EventVisionNode("spatter", "spatter_image", options)
  {
    cell_size_ = static_cast<int>(declare_parameter("cell_size", 7));
    RCLCPP_INFO(get_logger(), "spatter tracking: %.0f fps, cell %d px", fps(), cell_size_);
  }

  ~SpatterTracking() override { stopFrameThread(); }

protected:
  void onInit(uint16_t width, uint16_t height) override
  {
    const Metavision::SpatterTrackerAlgorithmConfig config(cell_size_, cell_size_);
    tracker_ = std::make_unique<Metavision::SpatterTrackerAlgorithm>(width, height, config);
    frame_gen_ = std::make_unique<Metavision::OnDemandFrameGenerationAlgorithm>(
      width, height, accTimeUs());
    RCLCPP_INFO(get_logger(), "pipeline initialized for %ux%u", width, height);
  }

  void processEvents(const std::vector<Metavision::EventCD> & events) override
  {
    clusters_.clear();
    if (events.empty()) {
      return;
    }
    // Ingest, then read back the current clusters at the latest event time.
    tracker_->process_events(events.begin(), events.end(), events.back().t, clusters_);
  }

  // Keep the latest box per cluster id, drop ones unseen recently (id < 0 are
  // untracked detections -- skip them).
  void stageResults() override
  {
    if (clusters_.empty()) {
      return;
    }
    const Metavision::timestamp now = clusters_.back().t;
    for (const auto & c : clusters_) {
      if (c.id >= 0) {
        latest_[c.id] = c;
      }
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
    work_.clear();
    for (const auto & kv : latest_) {
      work_.push_back(kv.second);
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
    if (!work_.empty()) {
      Metavision::draw_tracking_results(ts, work_.cbegin(), work_.cend(), frame);
    }
    return true;
  }

private:
  static constexpr Metavision::timestamp kPruneUs = 100000;

  int cell_size_{7};
  std::unique_ptr<Metavision::SpatterTrackerAlgorithm> tracker_;          // sub thread
  std::vector<Metavision::EventSpatterCluster> clusters_;                 // sub thread
  std::map<int, Metavision::EventSpatterCluster> latest_;                 // shared (mutex())
  std::vector<Metavision::EventSpatterCluster> work_;                     // frame thread
  std::unique_ptr<Metavision::OnDemandFrameGenerationAlgorithm> frame_gen_;  // frame thread
};

}  // namespace evk4_sdk_advanced

RCLCPP_COMPONENTS_REGISTER_NODE(evk4_sdk_advanced::SpatterTracking)

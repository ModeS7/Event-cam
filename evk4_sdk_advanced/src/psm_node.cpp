// Particle Size Monitoring (PSM) on the EVK4 event stream: counts objects crossing
// a set of horizontal lines and estimates each particle's size, overlaying the
// count + line clusters + particle tracks on the event image. For conveyor/channel
// QC of fast-moving objects. Subclass of EventVisionNode; SDK via process_events.

#include "evk4_sdk_advanced/event_vision_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <metavision/sdk/analytics/algorithms/psm_algorithm.h>
#include <metavision/sdk/analytics/utils/counting_drawing_helper.h>
#include <metavision/sdk/analytics/utils/line_cluster_drawing_helper.h>
#include <metavision/sdk/analytics/utils/line_particle_track_drawing_helper.h>
#include <metavision/sdk/core/algorithms/on_demand_frame_generation_algorithm.h>

#include <algorithm>
#include <memory>
#include <vector>

namespace evk4_sdk_advanced
{

class Psm : public EventVisionNode
{
public:
  explicit Psm(const rclcpp::NodeOptions & options)
  : EventVisionNode("psm", "psm_image", options)
  {
    min_y_ = static_cast<int>(declare_parameter("min_y", 150));
    max_y_ = static_cast<int>(declare_parameter("max_y", 570));
    num_lines_ = static_cast<int>(declare_parameter("num_lines", 6));
    cluster_ths_ = static_cast<int>(declare_parameter("cluster_ths", 3));
    num_clusters_ths_ = static_cast<int>(declare_parameter("num_clusters_ths", 4));
    precision_time_us_ = static_cast<int>(declare_parameter("precision_time_us", 1000));
    dt_first_match_us_ = static_cast<int>(declare_parameter("dt_first_match_us", 10000));
    is_going_up_ = declare_parameter("is_going_up", false);
    RCLCPP_INFO(
      get_logger(), "particle size monitoring: %.0f fps, %d lines in y[%d,%d]",
      fps(), num_lines_, min_y_, max_y_);
  }

  ~Psm() override { stopFrameThread(); }

protected:
  void onInit(uint16_t width, uint16_t height) override
  {
    rows_.clear();
    if (num_lines_ <= 1) {
      rows_.push_back((min_y_ + max_y_) / 2);
    } else {
      for (int i = 0; i < num_lines_; ++i) {
        rows_.push_back(min_y_ + i * (max_y_ - min_y_) / (num_lines_ - 1));
      }
    }
    const int bitsets = std::max(1, static_cast<int>(accTimeUs()) / std::max(1, precision_time_us_));
    const Metavision::LineClusterTrackingConfig detection_config(
      static_cast<unsigned int>(bitsets), static_cast<unsigned int>(cluster_ths_),
      static_cast<unsigned int>(num_clusters_ths_), 1u, 1.0f, 5.0f);
    const Metavision::LineParticleTrackingConfig tracking_config(
      !is_going_up_, static_cast<unsigned int>(dt_first_match_us_), 1.0, 0.5);
    psm_ = std::make_unique<Metavision::PsmAlgorithm>(
      width, height, rows_, detection_config, tracking_config);
    count_helper_ = std::make_unique<Metavision::CountingDrawingHelper>(rows_);
    cluster_helper_ = std::make_unique<Metavision::LineClusterDrawingHelper>();
    track_helper_ = std::make_unique<Metavision::LineParticleTrackDrawingHelper>(
      static_cast<int>(accTimeUs()) * 5);  // contour persistence on the display
    frame_gen_ = std::make_unique<Metavision::OnDemandFrameGenerationAlgorithm>(
      width, height, accTimeUs());
    RCLCPP_INFO(get_logger(), "pipeline initialized for %ux%u", width, height);
  }

  void processEvents(const std::vector<Metavision::EventCD> & events) override
  {
    if (events.empty()) {
      return;
    }
    psm_->process_events(events.begin(), events.end(), events.back().t, tracks_, clusters_);
  }

  void stageResults() override
  {
    // LineParticleTrackingOutput is move-only (its buffer is an OptimVector); move
    // it across to the frame thread. global_counter is read first (it's cumulative
    // in the algorithm, refilled on the next process_events into the moved buffer).
    staged_count_ = tracks_.global_counter;
    staged_tracks_ = std::move(tracks_);
    staged_clusters_ = clusters_;
  }

  void swapResults() override
  {
    work_count_ = staged_count_;
    std::swap(staged_tracks_, work_tracks_);
    std::swap(staged_clusters_, work_clusters_);
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
    count_helper_->draw(ts, work_count_, frame);
    cluster_helper_->draw(frame, work_clusters_.cbegin(), work_clusters_.cend());
    track_helper_->draw(ts, frame, work_tracks_.buffer.cbegin(), work_tracks_.buffer.cend());
    return true;
  }

private:
  int min_y_{150}, max_y_{570}, num_lines_{6};
  int cluster_ths_{3}, num_clusters_ths_{4};
  int precision_time_us_{1000}, dt_first_match_us_{10000};
  bool is_going_up_{false};
  std::vector<int> rows_;
  std::unique_ptr<Metavision::PsmAlgorithm> psm_;                              // sub thread
  Metavision::LineParticleTrackingOutput tracks_;                             // sub thread
  Metavision::PsmAlgorithm::LineClustersOutput clusters_;                     // sub thread
  int staged_count_{0}, work_count_{0};                                       // count (cumulative)
  Metavision::LineParticleTrackingOutput staged_tracks_, work_tracks_;        // staged / frame
  Metavision::PsmAlgorithm::LineClustersOutput staged_clusters_, work_clusters_;
  std::unique_ptr<Metavision::CountingDrawingHelper> count_helper_;           // frame thread
  std::unique_ptr<Metavision::LineClusterDrawingHelper> cluster_helper_;      // frame thread
  std::unique_ptr<Metavision::LineParticleTrackDrawingHelper> track_helper_;  // frame thread
  std::unique_ptr<Metavision::OnDemandFrameGenerationAlgorithm> frame_gen_;   // frame thread
};

}  // namespace evk4_sdk_advanced

RCLCPP_COMPONENTS_REGISTER_NODE(evk4_sdk_advanced::Psm)

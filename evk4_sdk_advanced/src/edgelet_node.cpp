// 2D edgelet detection + tracking on the EVK4 event stream: finds short oriented
// edge segments ("edgelets") in a time surface and tracks them frame to frame,
// overlaid on the event image. A subclass of EventVisionNode (the real-time
// decode + threading harness); the SDK is consumed via process(). Needs the SDK
// `cv3d` module (built with -DUSE_SOPHUS=ON). Single camera, no calibration --
// the 2D building block underneath the SDK's monocular 3D model tracking.

#include "evk4_sdk_advanced/event_vision_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <metavision/sdk/core/algorithms/on_demand_frame_generation_algorithm.h>
#include <metavision/sdk/core/utils/mostrecent_timestamp_buffer.h>
#include <metavision/sdk/cv/algorithms/spatio_temporal_contrast_algorithm.h>
#include <metavision/sdk/cv3d/algorithms/edgelet_2d_detection_algorithm.h>
#include <metavision/sdk/cv3d/algorithms/edgelet_2d_tracking_algorithm.h>
#include <metavision/sdk/cv3d/events/event_edgelet_2d.h>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <memory>
#include <vector>

namespace evk4_sdk_advanced
{
using Metavision::EventCD;
using Metavision::EventEdgelet2d;

class EdgeletTracking : public EventVisionNode
{
public:
  explicit EdgeletTracking(const rclcpp::NodeOptions & options)
  : EventVisionNode("edgelet", "edgelet_image", options)
  {
    // Clamp to >=1: a 0 (or negative) cell size divides by zero per event (SIGFPE,
    // a hard crash that bypasses clean shutdown) and builds an invalid grid.
    grid_cell_ = std::max(1, static_cast<int>(declare_parameter("grid_cell_size", 16)));
    stc_threshold_us_ = static_cast<int>(declare_parameter("stc_threshold_us", 5000));
    RCLCPP_INFO(
      get_logger(), "edgelet tracking: %.0f fps, grid %d px, STC %d us",
      fps(), grid_cell_, stc_threshold_us_);
  }

  ~EdgeletTracking() override { stopFrameThread(); }  // join before our members die

protected:
  void onInit(uint16_t width, uint16_t height) override
  {
    time_surface_ = std::make_unique<Metavision::MostRecentTimestampBuffer>(height, width, 2);
    time_surface_->set_to(0);
    stc_ = std::make_unique<Metavision::SpatioTemporalContrastAlgorithm>(
      width, height, stc_threshold_us_);
    grid_w_ = static_cast<int>(std::ceil(width / static_cast<float>(grid_cell_)));
    grid_h_ = static_cast<int>(std::ceil(height / static_cast<float>(grid_cell_)));
    mask_ = cv::Mat(grid_h_, grid_w_, CV_8UC1);
    frame_gen_ = std::make_unique<Metavision::OnDemandFrameGenerationAlgorithm>(
      width, height, accTimeUs());
    RCLCPP_INFO(get_logger(), "pipeline initialized for %ux%u", width, height);
  }

  // Sub thread (no lock): STC-filter, update the time surface, track the live
  // edgelets, then detect new ones in grid cells that have none.
  void processEvents(const std::vector<EventCD> & events) override
  {
    if (events.empty()) {
      return;
    }
    filtered_.clear();
    stc_->process_events(events.cbegin(), events.cend(), std::back_inserter(filtered_));
    if (filtered_.empty()) {
      return;
    }
    const Metavision::timestamp ts = filtered_.back().t;
    for (const auto & e : filtered_) {
      time_surface_->at(e.y, e.x, e.p) = e.t;
    }

    // Track the edgelets carried over from the previous packet.
    const auto n = edgelets_.size();
    statuses_.assign(n, false);
    tmp_.resize(n);
    auto tracked_end =
      trk_.process(*time_surface_, ts, edgelets_.cbegin(), edgelets_.cend(), tmp_.begin(),
        statuses_.begin());
    tmp_.resize(std::distance(tmp_.begin(), tracked_end));
    std::swap(edgelets_, tmp_);  // edgelets_ now holds only what was tracked

    // Mask the grid cells that already hold a tracked edgelet, then collect the
    // most recent event in each still-empty cell as a detection candidate.
    mask_.setTo(0);
    for (const auto & e : edgelets_) {
      const int cx = static_cast<int>(e.ctr2_(0)) / grid_cell_;
      const int cy = static_cast<int>(e.ctr2_(1)) / grid_cell_;
      if (cx >= 0 && cy >= 0 && cx < grid_w_ && cy < grid_h_) {
        mask_.at<uchar>(cy, cx) = 255;
      }
    }
    cand_.clear();
    for (auto it = filtered_.crbegin(); it != filtered_.crend(); ++it) {
      const int cx = it->x / grid_cell_;
      const int cy = it->y / grid_cell_;
      if (mask_.at<uchar>(cy, cx) == 0) {
        mask_.at<uchar>(cy, cx) = 255;
        cand_.emplace_back(*it);
      }
    }

    tmp_.resize(cand_.size());
    auto det_end = det_.process(*time_surface_, cand_.cbegin(), cand_.cend(), tmp_.begin());
    tmp_.resize(std::distance(tmp_.begin(), det_end));
    edgelets_.insert(edgelets_.cend(), tmp_.cbegin(), tmp_.cend());
  }

  void stageResults() override { staged_ = edgelets_; }
  void swapResults() override { std::swap(work_, staged_); }

  bool renderFrame(
    const std::vector<EventCD> & events, Metavision::timestamp ts, cv::Mat & frame) override
  {
    if (events.empty()) {
      return false;
    }
    frame_gen_->process_events(events.begin(), events.end());
    frame_gen_->generate(ts, frame);
    for (const auto & e : work_) {
      const cv::Point2f c(e.ctr2_(0), e.ctr2_(1));
      const cv::Point2f d(e.unit_dir2_(0), e.unit_dir2_(1));
      cv::line(frame, c + 4.f * d, c - 4.f * d, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }
    return true;
  }

private:
  int grid_cell_{16};
  int stc_threshold_us_{5000};
  int grid_w_{0}, grid_h_{0};

  std::unique_ptr<Metavision::MostRecentTimestampBuffer> time_surface_;  // sub thread
  std::unique_ptr<Metavision::SpatioTemporalContrastAlgorithm> stc_;     // sub thread
  Metavision::Edgelet2dDetectionAlgorithm det_;                          // sub thread
  Metavision::Edgelet2dTrackingAlgorithm trk_;                          // sub thread
  cv::Mat mask_;                                                        // sub thread
  std::vector<EventCD> filtered_, cand_;                               // sub thread
  std::vector<EventEdgelet2d> edgelets_, tmp_;                         // sub thread (persistent)
  std::vector<bool> statuses_;                                        // sub thread
  std::vector<EventEdgelet2d> staged_, work_;                        // staged (mutex) / frame
  std::unique_ptr<Metavision::OnDemandFrameGenerationAlgorithm> frame_gen_;  // frame thread
};

}  // namespace evk4_sdk_advanced

RCLCPP_COMPONENTS_REGISTER_NODE(evk4_sdk_advanced::EdgeletTracking)

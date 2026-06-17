// Event-level undistortion (lens rectification) on the EVK4 event stream.
//
// image_proc rectifies the rendered image_raw; it cannot rectify the event
// stream, because events are not an image. This pipeline does that: it loads the
// SAME standard camera_info YAML that evk4_calibration produces (and evk4_bringup
// feeds to image_proc), builds the SDK's pinhole CameraGeometry from its K +
// plumb_bob distortion, precomputes a distorted->undistorted pixel LUT once, and
// remaps every event to its undistorted position. The output is the rectified
// event view, published as <name>_image. Subclass of EventVisionNode.
//
// On the EVK4 kit's near-distortion-free 8 mm lens the rectified frame looks
// nearly identical to the raw one -- that is correct, the displacement is only a
// few pixels (see docs/calibration.md). With the zero-distortion placeholder
// calibration the LUT is the identity and this is a pass-through.

#include "evk4_sdk_advanced/event_vision_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <camera_calibration_parsers/parse.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <metavision/sdk/core/algorithms/on_demand_frame_generation_algorithm.h>
#include <metavision/sdk/cv/utils/camera_geometry.h>
#include <metavision/sdk/cv/utils/camera_geometry_helpers.h>
#include <metavision/sdk/cv/utils/pinhole_camera_model.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace evk4_sdk_advanced
{
using PinholeModel = Metavision::PinholeCameraModel<float>;
using PinholeGeometry = Metavision::CameraGeometry<PinholeModel>;

class Undistortion : public EventVisionNode
{
public:
  explicit Undistortion(const rclcpp::NodeOptions & options)
  : EventVisionNode("undistortion", "undistortion_image", options)
  {
    const auto url = declare_parameter<std::string>("calibration_url", "");
    if (url.empty()) {
      throw std::runtime_error(
        "undistortion requires calibration_url (the camera_info YAML from evk4_calibration)");
    }
    sensor_msgs::msg::CameraInfo info;
    std::string camera_name;
    if (!camera_calibration_parsers::readCalibration(url, camera_name, info)) {
      throw std::runtime_error("cannot read calibration file: " + url);
    }
    if (info.distortion_model != "plumb_bob" && info.distortion_model != "rational_polynomial") {
      RCLCPP_WARN(
        get_logger(), "distortion_model '%s' is not plumb_bob; using the first 5 coefficients",
        info.distortion_model.c_str());
    }
    K_.assign(info.k.begin(), info.k.end());  // 9 floats, row-major [fx 0 cx 0 fy cy 0 0 1]
    D_.assign(5, 0.0f);                        // plumb_bob: k1 k2 p1 p2 k3
    for (size_t i = 0; i < D_.size() && i < info.d.size(); ++i) {
      D_[i] = static_cast<float>(info.d[i]);
    }
    calib_w_ = static_cast<int>(info.width);
    calib_h_ = static_cast<int>(info.height);
    RCLCPP_INFO(
      get_logger(), "loaded calibration '%s' (%dx%d) from %s",
      camera_name.c_str(), calib_w_, calib_h_, url.c_str());
  }

  ~Undistortion() override { stopFrameThread(); }

protected:
  void onInit(uint16_t width, uint16_t height) override
  {
    W_ = width;
    H_ = height;
    if (calib_w_ != W_ || calib_h_ != H_) {
      RCLCPP_WARN(
        get_logger(), "calibration is %dx%d but the sensor is %ux%u -- using it anyway",
        calib_w_, calib_h_, width, height);
    }
    geometry_ = std::make_unique<PinholeGeometry>(PinholeModel(calib_w_, calib_h_, K_, D_));
    // Precompute the distorted->undistorted pixel map once, so each event is a
    // single lookup on the frame thread (the inverse-distortion solve is too
    // expensive to run per event). ~1 M cells; a one-time startup cost.
    RCLCPP_INFO(get_logger(), "building undistortion map (%ux%u)...", width, height);
    map_.create(H_, W_);
    for (int y = 0; y < H_; ++y) {
      for (int x = 0; x < W_; ++x) {
        const cv::Point2f dist_pt(static_cast<float>(x), static_cast<float>(y));
        cv::Matx21f undist_norm;
        std::vector<float> undist_img(2);
        Metavision::img_to_undist_norm(*geometry_, dist_pt, undist_norm);
        Metavision::undist_norm_to_undist_img(*geometry_, undist_norm, undist_img);
        const int ux = cvRound(undist_img[0]);
        const int uy = cvRound(undist_img[1]);
        map_(y, x) = (ux >= 0 && ux < W_ && uy >= 0 && uy < H_)
          ? cv::Vec2s(static_cast<short>(ux), static_cast<short>(uy))
          : cv::Vec2s(-1, -1);
      }
    }
    frame_gen_ = std::make_unique<Metavision::OnDemandFrameGenerationAlgorithm>(
      W_, H_, accTimeUs());
    RCLCPP_INFO(get_logger(), "pipeline initialized for %ux%u", width, height);
  }

  // Undistortion is a pure render-time remap -- no per-packet algorithm state.
  void processEvents(const std::vector<Metavision::EventCD> &) override {}
  void stageResults() override {}
  void swapResults() override {}

  bool renderFrame(
    const std::vector<Metavision::EventCD> & events, Metavision::timestamp ts,
    cv::Mat & frame) override
  {
    if (events.empty()) {
      return false;
    }
    undist_.clear();
    undist_.reserve(events.size());
    for (const auto & ev : events) {
      if (ev.x >= W_ || ev.y >= H_) {
        continue;
      }
      const cv::Vec2s & uv = map_(ev.y, ev.x);
      if (uv[0] < 0) {
        continue;  // undistorted position falls outside the sensor -> drop
      }
      undist_.emplace_back(
        static_cast<unsigned short>(uv[0]), static_cast<unsigned short>(uv[1]), ev.p, ev.t);
    }
    frame_gen_->process_events(undist_.begin(), undist_.end());
    frame_gen_->generate(ts, frame);
    return true;
  }

private:
  std::vector<float> K_, D_;
  int calib_w_{0}, calib_h_{0};
  int W_{0}, H_{0};
  cv::Mat_<cv::Vec2s> map_;                                    // distorted->undistorted LUT
  std::unique_ptr<PinholeGeometry> geometry_;
  std::unique_ptr<Metavision::OnDemandFrameGenerationAlgorithm> frame_gen_;  // frame thread
  std::vector<Metavision::EventCD> undist_;                    // frame thread scratch
};

}  // namespace evk4_sdk_advanced

RCLCPP_COMPONENTS_REGISTER_NODE(evk4_sdk_advanced::Undistortion)

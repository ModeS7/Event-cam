// Publish CameraInfo matched to image_raw, so image_proc can rectify.
//
// Neither the driver nor the renderer publishes CameraInfo. This component
// loads a standard camera_info YAML (`calibration_url`) and republishes it
// with the SAME header as each incoming image_raw, so image_proc pairs them
// by exact timestamp. It is loaded into the camera container with
// intra-process communication: images arrive as pointers and only the
// 16-byte header is read, so the cost stays near zero at any frame rate.
#include <stdexcept>
#include <string>

#include <camera_calibration_parsers/parse.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace evk4_bringup
{
class CameraInfoPublisher : public rclcpp::Node
{
public:
  explicit CameraInfoPublisher(const rclcpp::NodeOptions & options)
  : Node("camera_info_publisher", options)
  {
    const auto url = this->declare_parameter<std::string>("calibration_url", "");
    if (url.empty()) {
      throw std::runtime_error("calibration_url parameter is required");
    }
    std::string cameraName;
    if (!camera_calibration_parsers::readCalibration(url, cameraName, info_)) {
      throw std::runtime_error("cannot read calibration file: " + url);
    }
    pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw", 10,
      [this](sensor_msgs::msg::Image::ConstSharedPtr img) {
        info_.header = img->header;  // same stamp and frame_id
        pub_->publish(info_);
      });
    RCLCPP_INFO_STREAM(
      this->get_logger(), "publishing camera_info from " << url << " (" << info_.width
                                                         << "x" << info_.height << ")");
  }

private:
  sensor_msgs::msg::CameraInfo info_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};
}  // namespace evk4_bringup

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(evk4_bringup::CameraInfoPublisher)

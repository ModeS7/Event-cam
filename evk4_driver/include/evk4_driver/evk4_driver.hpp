// Prophesee EVK4 driver node on OpenEB / Metavision SDK. Opens the camera,
// forwards raw EVT3 as event_camera_msgs/EventPacket, configures the on-sensor
// facilities (ERC, Trail/STC, ROI, sync, trigger-in, AFK), exposes biases as
// live parameters, and offers the save_biases / save_settings services.
#ifndef EVK4_DRIVER__EVK4_DRIVER_HPP_
#define EVK4_DRIVER__EVK4_DRIVER_HPP_

#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <event_camera_msgs/msg/event_packet.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

// OpenEB is a subset of the full SDK; both define METAVISION_VERSION via CMake.
// The Camera class moved from sdk/driver to sdk/stream in Metavision 5.
#if defined(METAVISION_VERSION) && METAVISION_VERSION < 5
#include <metavision/sdk/driver/camera.h>
#else
#include <metavision/sdk/stream/camera.h>
#endif

namespace evk4_driver
{
class EVK4Driver : public rclcpp::Node
{
public:
  explicit EVK4Driver(const rclcpp::NodeOptions & options);
  ~EVK4Driver() override;

private:
  void startCamera();
  void loadSettings();   // camera state / pixel masks from the settings file
  void loadBiasFile();   // biases from a .bias file
  void declareBiases();  // expose each sensor bias as a live int parameter

  // On-sensor facility configuration, applied after open and before start.
  void configureSensor();
  void configureERC();
  void configureTrailFilter();
  void configureROI();
  void configureSync();
  void configureTriggerIn();
  void configureAFK();
  void configureERAF();         // event rate activity filter
  void configureDigitalCrop();
  void configureEventMask();

  // Live bias tuning: apply a changed bias parameter straight to the sensor.
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & params);

  void saveBiases(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void saveSettings(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // Called from the SDK raw-data thread with a chunk of raw EVT3 bytes.
  void rawDataCallback(const uint8_t * start, const uint8_t * end);

  using EventPacketMsg = event_camera_msgs::msg::EventPacket;

  rclcpp::Publisher<EventPacketMsg>::SharedPtr eventPub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr saveBiasesSrv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr saveSettingsSrv_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr paramCbHandle_;
  Metavision::Camera cam_;

  std::string serial_;
  std::string biasFile_;
  std::string settingsFile_;
  std::string frameId_{"event_camera"};
  std::string encoding_{"evt3"};
  std::set<std::string> biasNames_;
  uint32_t width_{0};
  uint32_t height_{0};
  uint32_t seq_{0};

  // Outgoing packet, accumulated across raw-data callbacks until a threshold.
  EventPacketMsg::UniquePtr msg_;
  uint64_t lastMessageTime_{0};
  uint64_t messageThresholdTime_{1000000};       // ns (1 ms default)
  size_t messageThresholdSize_{1000000000};      // bytes (~off by default)
  size_t reserveSize_{0};
};
}  // namespace evk4_driver

#endif  // EVK4_DRIVER__EVK4_DRIVER_HPP_

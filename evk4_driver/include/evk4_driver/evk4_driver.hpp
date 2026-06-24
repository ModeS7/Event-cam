// Prophesee EVK4 driver node on OpenEB / Metavision SDK. Opens the camera,
// forwards raw EVT3 as event_camera_msgs/EventPacket, configures the on-sensor
// facilities (ERC, Trail/STC, ROI, sync, trigger-in, AFK), exposes biases as
// live parameters, and offers the save_settings service.
#ifndef EVK4_DRIVER__EVK4_DRIVER_HPP_
#define EVK4_DRIVER__EVK4_DRIVER_HPP_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <event_camera_msgs/msg/event_packet.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

// OpenEB is a subset of the full SDK; both define METAVISION_VERSION via CMake.
// The Camera class moved from sdk/driver to sdk/stream in Metavision 5.
#if defined(METAVISION_VERSION) && METAVISION_VERSION < 5
#include <metavision/sdk/driver/camera.h>
#include <metavision/sdk/driver/file_config_hints.h>
#else
#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/stream/file_config_hints.h>
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

  void saveSettings(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  void processingThread();   // multithreaded-capture worker
  void printStats();         // per-second stats line
  // Packs a chunk of raw EVT3 bytes (captured at time t) into an EventPacket.
  void rawDataCallback(const uint8_t * start, const uint8_t * end, uint64_t t);

  using EventPacketMsg = event_camera_msgs::msg::EventPacket;

  rclcpp::Publisher<EventPacketMsg>::SharedPtr eventPub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr saveSettingsSrv_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr paramCbHandle_;
  Metavision::Camera cam_;

  std::string serial_;
  std::string file_;          // optional RAW recording to replay instead of a live camera
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

  // Optional multithreaded capture: the SDK raw-data callback enqueues bytes
  // and a worker thread packs/publishes EventPackets, so the SDK thread never
  // blocks on ROS at high event rates.
  struct RawChunk { std::vector<uint8_t> data; uint64_t t; };
  bool useMultithreading_{true};
  std::deque<RawChunk> queue_;
  size_t queuedBytes_{0};                          // bytes in queue_ (under queueMutex_)
  // Drop-oldest backstop so a consumer slower than the sensor cannot grow the
  // queue without bound (best-effort QoS already permits event loss).
  static constexpr size_t kMaxQueueBytes = 256 * 1024 * 1024;  // ~256 MB
  std::mutex queueMutex_;
  std::condition_variable queueCv_;
  std::thread workerThread_;
  std::atomic<bool> keepRunning_{true};

  // Per-second statistics (msgs/s, MB/s) on a wall timer; interval <=0 disables.
  double statsInterval_{1.0};
  rclcpp::TimerBase::SharedPtr statsTimer_;
  std::atomic<size_t> statMsgs_{0};
  std::atomic<size_t> statBytes_{0};
  // Camera runtime/USB errors surfaced by the SDK: per-interval + cumulative.
  std::atomic<size_t> statErrors_{0};
  std::atomic<uint64_t> totalErrors_{0};
  std::atomic<uint64_t> totalDropped_{0};          // raw chunks dropped on queue overflow
};
}  // namespace evk4_driver

#endif  // EVK4_DRIVER__EVK4_DRIVER_HPP_

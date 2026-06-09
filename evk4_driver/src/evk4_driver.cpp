#include "evk4_driver/evk4_driver.hpp"

#include <algorithm>
#include <chrono>
#include <utility>

#include <metavision/hal/facilities/i_antiflicker_module.h>

namespace evk4_driver
{

EVK4Driver::EVK4Driver(const rclcpp::NodeOptions & options)
: Node("event_camera", options)
{
  serial_ = this->declare_parameter<std::string>("serial", "");
  const double tThresh =
    this->declare_parameter<double>("event_message_time_threshold", 0.001);
  messageThresholdTime_ = static_cast<uint64_t>(tThresh * 1e9);
  afkEnabled_ = this->declare_parameter<bool>("afk_enabled", false);
  afkFreqLow_ = this->declare_parameter<int>("afk_freq_low_hz", 100);
  afkFreqHigh_ = this->declare_parameter<int>("afk_freq_high_hz", 120);
  afkMode_ = this->declare_parameter<std::string>("afk_mode", "band_stop");

  // Match the metavision_driver publisher QoS exactly so the existing
  // event_camera_renderer / codecs subscribe without a QoS mismatch.
  rclcpp::QoS qos(rclcpp::KeepLast(1000));
  qos.best_effort().durability_volatile();
  eventPub_ = this->create_publisher<EventPacketMsg>("~/events", qos);

  startCamera();
}

EVK4Driver::~EVK4Driver()
{
  try {
    if (cam_.is_running()) {
      cam_.stop();
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN_STREAM(this->get_logger(), "error stopping camera: " << e.what());
  }
}

void EVK4Driver::startCamera()
{
  Metavision::DeviceConfig deviceConfig;
  deviceConfig.set_format("EVT3");
  try {
    if (!serial_.empty()) {
      cam_ = Metavision::Camera::from_serial(serial_, deviceConfig);
    } else {
      cam_ = Metavision::Camera::from_first_available(deviceConfig);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "failed to open camera: " << e.what());
    throw;
  }

  const auto & g = cam_.geometry();
  width_ = static_cast<uint32_t>(g.get_width());
  height_ = static_cast<uint32_t>(g.get_height());
  RCLCPP_INFO_STREAM(this->get_logger(), "opened EVK4 " << width_ << "x" << height_);

  configureAFK();

  // Raw EVT3 bytes straight from the sensor -> EventPacket (zero decode).
  cam_.raw_data().add_callback(
    [this](const uint8_t * data, size_t size) {
      this->rawDataCallback(data, data + size);
    });

  cam_.start();
  RCLCPP_INFO(this->get_logger(), "camera started, publishing evt3 EventPackets on ~/events");
}

void EVK4Driver::configureAFK()
{
  if (!afkEnabled_) {
    return;
  }
  auto * afk = cam_.get_device().get_facility<Metavision::I_AntiFlickerModule>();
  if (afk == nullptr) {
    RCLCPP_WARN(this->get_logger(), "anti-flicker (AFK) not available on this device");
    return;
  }
  afk->set_frequency_band(
    static_cast<uint32_t>(afkFreqLow_), static_cast<uint32_t>(afkFreqHigh_));
  afk->set_filtering_mode(
    afkMode_ == "band_pass" ? Metavision::I_AntiFlickerModule::BAND_PASS
                            : Metavision::I_AntiFlickerModule::BAND_STOP);
  afk->enable(true);
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "AFK enabled [" << afkFreqLow_ << "-" << afkFreqHigh_ << " Hz, " << afkMode_ << "]");
}

void EVK4Driver::rawDataCallback(const uint8_t * start, const uint8_t * end)
{
  // Lazy: only build packets while something is subscribed.
  if (eventPub_->get_subscription_count() == 0) {
    msg_.reset();
    return;
  }
  const uint64_t t = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();

  if (!msg_) {
    msg_ = std::make_unique<EventPacketMsg>();
    msg_->header.frame_id = frameId_;
    msg_->header.stamp = rclcpp::Time(static_cast<int64_t>(t), RCL_SYSTEM_TIME);
    msg_->time_base = 0;  // not used for evt3
    msg_->encoding = encoding_;
    msg_->seq = seq_++;
    msg_->width = width_;
    msg_->height = height_;
    msg_->events.reserve(reserveSize_);
  }

  msg_->events.insert(msg_->events.end(), start, end);

  if (t - lastMessageTime_ > messageThresholdTime_ ||
    msg_->events.size() > messageThresholdSize_)
  {
    reserveSize_ = std::max(reserveSize_, msg_->events.size());
    eventPub_->publish(std::move(msg_));
    msg_.reset();
    lastMessageTime_ = t;
  }
}

}  // namespace evk4_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(evk4_driver::EVK4Driver)

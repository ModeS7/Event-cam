#include "evk4_driver/evk4_driver.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <map>
#include <utility>
#include <vector>

#include <metavision/hal/facilities/i_antiflicker_module.h>
#include <metavision/hal/facilities/i_camera_synchronization.h>
#include <metavision/hal/facilities/i_erc_module.h>
#include <metavision/hal/facilities/i_event_trail_filter_module.h>
#include <metavision/hal/facilities/i_ll_biases.h>
#include <metavision/hal/facilities/i_roi.h>
#include <metavision/hal/facilities/i_trigger_in.h>

namespace evk4_driver
{
using std::placeholders::_1;
using std::placeholders::_2;

EVK4Driver::EVK4Driver(const rclcpp::NodeOptions & options)
: Node("event_camera", options)
{
  serial_ = this->declare_parameter<std::string>("serial", "");
  frameId_ = this->declare_parameter<std::string>("frame_id", "event_camera_optical_frame");
  biasFile_ = this->declare_parameter<std::string>("bias_file", "");
  settingsFile_ = this->declare_parameter<std::string>("settings", "");
  const double tThresh =
    this->declare_parameter<double>("event_message_time_threshold", 0.001);
  messageThresholdTime_ = static_cast<uint64_t>(tThresh * 1e9);

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

  loadSettings();
  loadBiasFile();
  declareBiases();

  saveBiasesSrv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/save_biases", std::bind(&EVK4Driver::saveBiases, this, _1, _2));
  saveSettingsSrv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/save_settings", std::bind(&EVK4Driver::saveSettings, this, _1, _2));

  configureSensor();

  // Register AFTER all parameters are declared so declaration does not fire it.
  paramCbHandle_ = this->add_on_set_parameters_callback(
    std::bind(&EVK4Driver::onSetParameters, this, _1));

  // Raw EVT3 bytes straight from the sensor -> EventPacket (zero decode).
  cam_.raw_data().add_callback(
    [this](const uint8_t * data, size_t size) {
      this->rawDataCallback(data, data + size);
    });

  cam_.start();
  RCLCPP_INFO(this->get_logger(), "camera started, publishing evt3 EventPackets on ~/events");
}

void EVK4Driver::loadSettings()
{
  if (settingsFile_.empty()) {
    return;
  }
  try {
    if (!cam_.load(settingsFile_)) {
      RCLCPP_WARN_STREAM(this->get_logger(), "could not load settings from: " << settingsFile_);
    } else {
      RCLCPP_INFO_STREAM(this->get_logger(), "loaded camera settings from: " << settingsFile_);
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN_STREAM(this->get_logger(), "loading settings failed: " << e.what());
  }
}

void EVK4Driver::loadBiasFile()
{
  if (biasFile_.empty()) {
    return;
  }
  auto * biases = cam_.get_device().get_facility<Metavision::I_LL_Biases>();
  if (biases == nullptr) {
    return;
  }
  try {
    biases->load_from_file(biasFile_);
    RCLCPP_INFO_STREAM(this->get_logger(), "loaded bias file: " << biasFile_);
  } catch (const std::exception & e) {
    RCLCPP_WARN_STREAM(
      this->get_logger(), "reading bias file failed: " << e.what() << "; using defaults");
  }
}

void EVK4Driver::declareBiases()
{
  auto * biases = cam_.get_device().get_facility<Metavision::I_LL_Biases>();
  if (biases == nullptr) {
    RCLCPP_WARN(this->get_logger(), "biases not available on this device");
    return;
  }
  const auto all = biases->get_all_biases();
  for (const auto & b : all) {
    if (b.first == "bias_diff") {
      continue;  // computed reference, not independently settable
    }
    biasNames_.insert(b.first);
    this->declare_parameter<int>(b.first, b.second);
  }
  RCLCPP_INFO_STREAM(
    this->get_logger(), "exposed " << biasNames_.size() << " tunable biases (ros2 param set)");
}

rcl_interfaces::msg::SetParametersResult EVK4Driver::onSetParameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;
  auto * biases = cam_.get_device().get_facility<Metavision::I_LL_Biases>();
  for (const auto & p : params) {
    if (biasNames_.count(p.get_name()) == 0 || biases == nullptr) {
      continue;  // not a live bias -> accept without side effect
    }
    const int val = static_cast<int>(p.as_int());
    if (!biases->set(p.get_name(), val)) {
      RCLCPP_WARN_STREAM(this->get_logger(), "cannot set " << p.get_name() << " to " << val);
    } else {
      RCLCPP_INFO_STREAM(
        this->get_logger(), "bias " << p.get_name() << " -> " << biases->get(p.get_name()));
    }
  }
  return res;
}

void EVK4Driver::saveBiases(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  if (biasFile_.empty()) {
    res->success = false;
    res->message = "no bias file specified at startup";
    return;
  }
  auto * biases = cam_.get_device().get_facility<Metavision::I_LL_Biases>();
  try {
    biases->save_to_file(biasFile_);
    res->success = true;
    res->message = "biases written to " + biasFile_;
  } catch (const std::exception & e) {
    res->success = false;
    res->message = std::string("bias file write failed: ") + e.what();
  }
}

void EVK4Driver::saveSettings(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  if (settingsFile_.empty()) {
    res->success = false;
    res->message = "no settings file specified at startup";
    return;
  }
  try {
    cam_.save(settingsFile_);
    res->success = true;
    res->message = "settings written to " + settingsFile_;
  } catch (const std::exception & e) {
    res->success = false;
    res->message = std::string("settings write failed: ") + e.what();
  }
}

void EVK4Driver::configureSensor()
{
  configureERC();
  configureTrailFilter();
  configureROI();
  configureSync();
  configureTriggerIn();
  configureAFK();
}

void EVK4Driver::configureERC()
{
  const std::string mode = this->declare_parameter<std::string>("erc_mode", "na");
  const int rate = this->declare_parameter<int>("erc_rate", 100000000);
  if (mode != "enabled" && mode != "disabled") {
    return;  // "na" -> leave the sensor default untouched
  }
  auto * erc = cam_.get_device().get_facility<Metavision::I_ErcModule>();
  if (erc == nullptr) {
    RCLCPP_WARN(this->get_logger(), "ERC (event rate control) not available on this device");
    return;
  }
  erc->enable(mode == "enabled");
  erc->set_cd_event_rate(rate);
  RCLCPP_INFO_STREAM(this->get_logger(), "ERC " << mode << " @ " << rate << " ev/s");
}

void EVK4Driver::configureTrailFilter()
{
  const bool enabled = this->declare_parameter<bool>("trail_filter", false);
  const std::string type = this->declare_parameter<std::string>("trail_filter_type", "trail");
  const int threshold = this->declare_parameter<int>("trail_filter_threshold", 0);
  if (!enabled) {
    return;
  }
  auto * tf = cam_.get_device().get_facility<Metavision::I_EventTrailFilterModule>();
  if (tf == nullptr) {
    RCLCPP_WARN(this->get_logger(), "trail/STC filter not available on this device");
    return;
  }
  static const std::map<std::string, Metavision::I_EventTrailFilterModule::Type> typeMap = {
    {"trail", Metavision::I_EventTrailFilterModule::Type::TRAIL},
    {"stc_cut_trail", Metavision::I_EventTrailFilterModule::Type::STC_CUT_TRAIL},
    {"stc_keep_trail", Metavision::I_EventTrailFilterModule::Type::STC_KEEP_TRAIL}};
  const auto it = typeMap.find(type);
  if (it == typeMap.end()) {
    RCLCPP_WARN_STREAM(this->get_logger(), "unknown trail_filter_type: " << type);
  } else {
    tf->set_type(it->second);
    tf->set_threshold(static_cast<uint32_t>(threshold));
  }
  tf->enable(true);
  RCLCPP_INFO_STREAM(
    this->get_logger(), "trail filter " << type << " threshold=" << threshold);
}

void EVK4Driver::configureROI()
{
  const std::vector<int64_t> roi =
    this->declare_parameter<std::vector<int64_t>>("roi", std::vector<int64_t>{});
  const bool roni = this->declare_parameter<bool>("roni", false);
  if (roi.empty()) {
    return;
  }
  if (roi.size() % 4 != 0) {
    RCLCPP_ERROR_STREAM(
      this->get_logger(), "roi must be a multiple of 4 [x,y,w,h,...], got " << roi.size());
    return;
  }
  auto * i_roi = cam_.get_device().get_facility<Metavision::I_ROI>();
  if (i_roi == nullptr) {
    RCLCPP_WARN(this->get_logger(), "ROI not available on this device");
    return;
  }
  std::vector<Metavision::I_ROI::Window> rects;
  for (size_t i = 0; i < roi.size(); i += 4) {
    Metavision::I_ROI::Window w;
    w.x = static_cast<int>(roi[i]);
    w.y = static_cast<int>(roi[i + 1]);
    w.width = static_cast<int>(roi[i + 2]);
    w.height = static_cast<int>(roi[i + 3]);
    rects.push_back(w);
  }
  i_roi->set_mode(roni ? Metavision::I_ROI::Mode::RONI : Metavision::I_ROI::Mode::ROI);
  i_roi->set_windows(rects);
  i_roi->enable(true);
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "ROI " << rects.size() << " window(s), mode " << (roni ? "RONI" : "ROI"));
}

void EVK4Driver::configureSync()
{
  const std::string mode = this->declare_parameter<std::string>("sync_mode", "standalone");
  auto * sync = cam_.get_device().get_facility<Metavision::I_CameraSynchronization>();
  if (sync == nullptr) {
    if (mode != "standalone") {
      RCLCPP_WARN_STREAM(this->get_logger(), "cannot set sync mode to: " << mode);
    }
    return;
  }
  if (mode == "standalone") {
    sync->set_mode_standalone();
  } else if (mode == "primary") {
    sync->set_mode_master();
  } else if (mode == "secondary") {
    sync->set_mode_slave();
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "invalid sync_mode: " << mode);
    throw std::runtime_error("invalid sync_mode: " + mode);
  }
  if (mode != "standalone") {
    RCLCPP_INFO_STREAM(this->get_logger(), "sync mode: " << mode);
  }
}

void EVK4Driver::configureTriggerIn()
{
  const std::string mode = this->declare_parameter<std::string>("trigger_in_mode", "disabled");
  if (mode == "disabled") {
    return;
  }
  static const std::map<std::string, Metavision::I_TriggerIn::Channel> channelMap = {
    {"external", Metavision::I_TriggerIn::Channel::Main},
    {"aux", Metavision::I_TriggerIn::Channel::Aux},
    {"loopback", Metavision::I_TriggerIn::Channel::Loopback}};
  auto * ti = cam_.get_device().get_facility<Metavision::I_TriggerIn>();
  if (ti == nullptr) {
    RCLCPP_WARN(this->get_logger(), "external trigger input not available on this device");
    return;
  }
  const auto it = channelMap.find(mode);
  if (it == channelMap.end()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "invalid trigger_in_mode: " << mode);
    return;
  }
  ti->enable(it->second);
  RCLCPP_INFO_STREAM(this->get_logger(), "external trigger input enabled: " << mode);
}

void EVK4Driver::configureAFK()
{
  const bool enabled = this->declare_parameter<bool>("afk_enabled", false);
  const int low = this->declare_parameter<int>("afk_freq_low_hz", 100);
  const int high = this->declare_parameter<int>("afk_freq_high_hz", 120);
  const std::string mode = this->declare_parameter<std::string>("afk_mode", "band_stop");
  if (!enabled) {
    return;
  }
  auto * afk = cam_.get_device().get_facility<Metavision::I_AntiFlickerModule>();
  if (afk == nullptr) {
    RCLCPP_WARN(this->get_logger(), "anti-flicker (AFK) not available on this device");
    return;
  }
  afk->set_frequency_band(static_cast<uint32_t>(low), static_cast<uint32_t>(high));
  afk->set_filtering_mode(
    mode == "band_pass" ? Metavision::I_AntiFlickerModule::BAND_PASS
                        : Metavision::I_AntiFlickerModule::BAND_STOP);
  afk->enable(true);
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "AFK enabled [" << low << "-" << high << " Hz, " << mode << "]");
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

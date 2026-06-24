#include "evk4_driver/evk4_driver.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <map>
#include <utility>
#include <vector>

#include <metavision/sdk/base/events/event_cd.h>

#include <metavision/hal/facilities/i_antiflicker_module.h>
#include <metavision/hal/facilities/i_camera_synchronization.h>
#include <metavision/hal/facilities/i_digital_crop.h>
#include <metavision/hal/facilities/i_digital_event_mask.h>
#include <metavision/hal/facilities/i_erc_module.h>
#include <metavision/hal/facilities/i_hw_identification.h>
#include <metavision/hal/facilities/i_event_rate_activity_filter_module.h>
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
  // Offline replay: point at a recorded RAW/EVT3 file to feed the pipeline
  // without a live camera (the "no camera" / driving-clip demos). Takes
  // precedence over serial.
  file_ = this->declare_parameter<std::string>("file", "");
  frameId_ = this->declare_parameter<std::string>("frame_id", "event_camera_optical_frame");
  settingsFile_ = this->declare_parameter<std::string>("settings", "");
  const double tThresh =
    this->declare_parameter<double>("event_message_time_threshold", 0.001);
  messageThresholdTime_ = static_cast<uint64_t>(tThresh * 1e9);
  useMultithreading_ = this->declare_parameter<bool>("use_multithreading", true);
  statsInterval_ = this->declare_parameter<double>("statistics_print_interval", 1.0);

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
  keepRunning_ = false;
  queueCv_.notify_all();
  if (workerThread_.joinable()) {
    workerThread_.join();
  }
}

void EVK4Driver::startCamera()
{
  try {
    if (!file_.empty()) {
      // Replay a recorded RAW: real_time_playback streams at the recorded rate
      // so downstream nodes (renderer, SDK pipelines) behave as with a live
      // camera. EVK4 recordings are already EVT3, so decoding is unchanged.
      cam_ = Metavision::Camera::from_file(
        file_, Metavision::FileConfigHints().real_time_playback(true));
      RCLCPP_INFO_STREAM(this->get_logger(), "replaying recorded events from: " << file_);
    } else {
      Metavision::DeviceConfig deviceConfig;
      deviceConfig.set_format("EVT3");
      if (!serial_.empty()) {
        cam_ = Metavision::Camera::from_serial(serial_, deviceConfig);
      } else {
        cam_ = Metavision::Camera::from_first_available(deviceConfig);
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "failed to open camera: " << e.what());
    throw;
  }

  const auto & g = cam_.geometry();
  width_ = static_cast<uint32_t>(g.get_width());
  height_ = static_cast<uint32_t>(g.get_height());
  auto * hwid = cam_.get_device().get_facility<Metavision::I_HW_Identification>();
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "opened EVK4 " << width_ << "x" << height_
                   << (hwid ? ", serial " + hwid->get_serial() : ""));

  loadSettings();
  declareBiases();

  saveSettingsSrv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/save_settings", std::bind(&EVK4Driver::saveSettings, this, _1, _2));

  configureSensor();

  // Register AFTER all parameters are declared so declaration does not fire it.
  paramCbHandle_ = this->add_on_set_parameters_callback(
    std::bind(&EVK4Driver::onSetParameters, this, _1));

  // Raw EVT3 bytes straight from the sensor -> EventPacket (zero decode).
  // Single-threaded: pack+publish inline. Multithreaded: enqueue fast and let
  // the worker pack+publish, so this SDK callback never blocks on ROS.
  cam_.raw_data().add_callback(
    [this](const uint8_t * data, size_t size) {
      const uint64_t t = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
      if (useMultithreading_) {
        {
          std::lock_guard<std::mutex> lock(queueMutex_);
          queue_.push_front(RawChunk{std::vector<uint8_t>(data, data + size), t});
          queuedBytes_ += size;
          // Bound memory under sustained overload: drop the OLDEST chunks rather
          // than grow without limit (keep at least the chunk just enqueued).
          while (queuedBytes_ > kMaxQueueBytes && queue_.size() > 1) {
            queuedBytes_ -= queue_.back().data.size();
            queue_.pop_back();
            totalDropped_.fetch_add(1, std::memory_order_relaxed);
          }
        }
        queueCv_.notify_one();
      } else {
        this->rawDataCallback(data, data + size, t);
      }
    });

  // Offline file replay only paces to real time when a DECODE callback is
  // registered -- the SDK derives the pace from decoded event timestamps
  // (camera_offline_raw.cpp gates the inter-buffer sleep on has_decode_callbacks).
  // We forward raw bytes and never otherwise decode, so without this the file
  // floods out as fast as it reads (~80x real time), ending before subscribers
  // (and the ML model load) are ready and overrunning best-effort QoS. Register
  // a no-op CD callback purely to engage that throttle.
  if (!file_.empty()) {
    cam_.cd().add_callback(
      [](const Metavision::EventCD *, const Metavision::EventCD *) {});
  }

  // Surface camera runtime / USB errors that would otherwise be silent: log a
  // warning and count them (reported in the per-second stats line).
  cam_.add_runtime_error_callback(
    [this](const Metavision::CameraException & e) {
      statErrors_.fetch_add(1, std::memory_order_relaxed);
      totalErrors_.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_WARN_STREAM(this->get_logger(), "camera runtime error: " << e.what());
    });

  if (statsInterval_ > 0.0) {
    statsTimer_ = this->create_wall_timer(
      std::chrono::duration<double>(statsInterval_),
      std::bind(&EVK4Driver::printStats, this));
  }

  cam_.start();
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "camera started, publishing evt3 EventPackets on ~/events"
      << (useMultithreading_ ? " (multithreaded)" : ""));

  // Start the capture worker LAST, after cam_.start() has succeeded -- nothing
  // after this point can throw, so no joinable std::thread can survive a
  // constructor exception (which would call std::terminate instead of the
  // catchable error the standalone node / component loader expects).
  if (useMultithreading_) {
    workerThread_ = std::thread(&EVK4Driver::processingThread, this);
  }
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
    // The sensor knows each bias's valid range; declare it on the param so
    // `ros2 param describe` reports it, rqt_reconfigure gets correctly
    // bounded sliders, and out-of-range values are rejected up front.
    rcl_interfaces::msg::ParameterDescriptor desc;
    Metavision::LL_Bias_Info info;
    if (biases->get_bias_info(b.first, info)) {
      const auto range = info.get_bias_range();
      rcl_interfaces::msg::IntegerRange ir;
      ir.from_value = range.first;
      ir.to_value = range.second;
      ir.step = 1;
      desc.integer_range.push_back(ir);
      desc.description = "sensor range " + std::to_string(range.first) + ".." +
        std::to_string(range.second);
      RCLCPP_INFO_STREAM(
        this->get_logger(), b.first << " = " << b.second << " [range " << range.first
                                    << ".." << range.second << "]");
    }
    // declare_parameter returns the override when one was given (params
    // YAML / launch), so biases can be set at startup like everything else
    // -- they reset to sensor defaults on every camera open otherwise.
    const int v = this->declare_parameter<int>(b.first, b.second, desc);
    if (v != b.second) {
      if (biases->set(b.first, v)) {
        RCLCPP_INFO_STREAM(this->get_logger(), "startup bias " << b.first << " = " << v);
      } else {
        RCLCPP_WARN_STREAM(
          this->get_logger(), "cannot set startup bias " << b.first << " to " << v);
      }
    }
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
  configureERAF();
  configureDigitalCrop();
  configureEventMask();
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

void EVK4Driver::configureERAF()
{
  const bool enabled = this->declare_parameter<bool>("eraf_enabled", false);
  const int ls = this->declare_parameter<int>("eraf_lower_start", 0);
  const int lp = this->declare_parameter<int>("eraf_lower_stop", 0);
  const int us = this->declare_parameter<int>("eraf_upper_start", 0);
  const int up = this->declare_parameter<int>("eraf_upper_stop", 0);
  if (!enabled) {
    return;
  }
  auto * eraf =
    cam_.get_device().get_facility<Metavision::I_EventRateActivityFilterModule>();
  if (eraf == nullptr) {
    RCLCPP_WARN(this->get_logger(), "event rate activity filter not available on this device");
    return;
  }
  // All four fields must be set together (partial init -> undefined thresholds).
  Metavision::I_EventRateActivityFilterModule::thresholds th;
  th.lower_bound_start = static_cast<uint32_t>(ls);
  th.lower_bound_stop = static_cast<uint32_t>(lp);
  th.upper_bound_start = static_cast<uint32_t>(us);
  th.upper_bound_stop = static_cast<uint32_t>(up);
  eraf->set_thresholds(th);
  eraf->enable(true);
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "ERAF enabled [lower " << ls << "/" << lp << ", upper " << us << "/" << up << " ev/s]");
}

void EVK4Driver::configureDigitalCrop()
{
  const bool enabled = this->declare_parameter<bool>("digital_crop_enabled", false);
  const std::vector<int64_t> region =
    this->declare_parameter<std::vector<int64_t>>("digital_crop_region", std::vector<int64_t>{});
  const bool resetOrigin = this->declare_parameter<bool>("digital_crop_reset_origin", false);
  if (!enabled) {
    return;
  }
  if (region.size() != 4) {
    RCLCPP_ERROR_STREAM(
      this->get_logger(),
      "digital_crop_region must be [x_start,y_start,x_end,y_end], got " << region.size());
    return;
  }
  auto * crop = cam_.get_device().get_facility<Metavision::I_DigitalCrop>();
  if (crop == nullptr) {
    RCLCPP_WARN(this->get_logger(), "digital crop not available on this device");
    return;
  }
  const Metavision::I_DigitalCrop::Region r{
    static_cast<uint32_t>(region[0]), static_cast<uint32_t>(region[1]),
    static_cast<uint32_t>(region[2]), static_cast<uint32_t>(region[3])};
  crop->set_window_region(r, resetOrigin);
  crop->enable(true);
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "digital crop [" << region[0] << "," << region[1] << " -> " << region[2] << "," << region[3]
                     << "]" << (resetOrigin ? " (origin reset)" : ""));
}

void EVK4Driver::configureEventMask()
{
  const std::vector<int64_t> pixels =
    this->declare_parameter<std::vector<int64_t>>("event_mask_pixels", std::vector<int64_t>{});
  if (pixels.empty()) {
    return;
  }
  if (pixels.size() % 2 != 0) {
    RCLCPP_ERROR_STREAM(
      this->get_logger(),
      "event_mask_pixels must be [x0,y0,x1,y1,...] pairs, got " << pixels.size());
    return;
  }
  auto * mask = cam_.get_device().get_facility<Metavision::I_DigitalEventMask>();
  if (mask == nullptr) {
    RCLCPP_WARN(this->get_logger(), "digital event mask not available on this device");
    return;
  }
  const auto & slots = mask->get_pixel_masks();
  const size_t requested = pixels.size() / 2;
  if (requested > slots.size()) {
    RCLCPP_WARN_STREAM(
      this->get_logger(),
      "requested " << requested << " masked pixels but only " << slots.size()
                   << " mask slots available; masking the first " << slots.size());
  }
  const size_t count = std::min(requested, slots.size());
  for (size_t i = 0; i < count; ++i) {
    slots[i]->set_mask(
      static_cast<uint32_t>(pixels[2 * i]), static_cast<uint32_t>(pixels[2 * i + 1]), true);
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "masked " << count << " pixel(s)");
}

void EVK4Driver::rawDataCallback(const uint8_t * start, const uint8_t * end, uint64_t t)
{
  // Lazy: only build packets while something is subscribed.
  if (eventPub_->get_subscription_count() == 0) {
    msg_.reset();
    return;
  }
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
    statBytes_.fetch_add(msg_->events.size(), std::memory_order_relaxed);
    statMsgs_.fetch_add(1, std::memory_order_relaxed);
    eventPub_->publish(std::move(msg_));
    msg_.reset();
    lastMessageTime_ = t;
  }
}

void EVK4Driver::processingThread()
{
  while (keepRunning_) {
    RawChunk chunk;
    {
      std::unique_lock<std::mutex> lock(queueMutex_);
      queueCv_.wait_for(
        lock, std::chrono::milliseconds(100),
        [this] { return !queue_.empty() || !keepRunning_; });
      if (queue_.empty()) {
        continue;
      }
      chunk = std::move(queue_.back());
      queue_.pop_back();
      queuedBytes_ -= chunk.data.size();
    }
    rawDataCallback(chunk.data.data(), chunk.data.data() + chunk.data.size(), chunk.t);
  }
}

void EVK4Driver::printStats()
{
  const double dt = statsInterval_ > 0.0 ? statsInterval_ : 1.0;
  const size_t msgs = statMsgs_.exchange(0, std::memory_order_relaxed);
  const size_t bytes = statBytes_.exchange(0, std::memory_order_relaxed);
  const size_t errs = statErrors_.exchange(0, std::memory_order_relaxed);
  const uint64_t totErrs = totalErrors_.load(std::memory_order_relaxed);
  const uint64_t totDropped = totalDropped_.load(std::memory_order_relaxed);
  size_t q = 0;
  if (useMultithreading_) {
    std::lock_guard<std::mutex> lock(queueMutex_);
    q = queue_.size();
  }
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    static_cast<long>(msgs / dt) << " msgs/s, " << (bytes / dt) / 1e6 << " MB/s"
      << (useMultithreading_ ? " (queue " + std::to_string(q) + ")" : "")
      << (totErrs ? " | errors: " + std::to_string(errs) + "/interval, " +
            std::to_string(totErrs) + " total" : "")
      << (totDropped ? " | dropped: " + std::to_string(totDropped) : ""));
}

}  // namespace evk4_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(evk4_driver::EVK4Driver)

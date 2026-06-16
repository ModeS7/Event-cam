// Reusable real-time harness for "events -> Metavision SDK algorithm -> ROS
// image" pipelines on the EVK4 event stream.
//
// It subscribes to an event topic, decodes EVT3 with event_camera_codecs, runs
// the subclass's algorithm INCREMENTALLY per packet on the subscription thread
// (SDK algorithms are streaming and go super-linear on large batches), and
// renders + publishes exactly one newest frame per wall-clock fps tick on a
// separate frame thread. The mutex guards only cheap staging swaps -- never the
// heavy render -- so event ingestion never stalls and packets are not dropped.
//
// A subclass implements four hooks: onInit (create algorithm + frame
// generator), processEvents (run the algorithm, stage results), swapResults
// (swap staged results, called under the lock), and renderFrame (draw the
// image). See optical_flow_node.cpp and tracking_node.cpp.

#ifndef EVK4_SDK_ADVANCED__EVENT_VISION_NODE_HPP_
#define EVK4_SDK_ADVANCED__EVENT_VISION_NODE_HPP_

#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_codecs/event_packet.h>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <metavision/sdk/base/events/event_cd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace evk4_sdk_advanced
{
using event_camera_codecs::EventPacket;

class EventVisionNode : public rclcpp::Node, public event_camera_codecs::EventProcessor
{
public:
  EventVisionNode(
    const std::string & node_name, const std::string & image_topic,
    const rclcpp::NodeOptions & options)
  : rclcpp::Node(node_name, options)
  {
    fps_ = declare_parameter("fps", 30.0);
    acc_time_us_ = static_cast<uint32_t>(declare_parameter("accumulation_time_us", 10000));
    debug_timing_ = declare_parameter("debug_timing", false);
    pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic, rclcpp::QoS(1));
    sub_ = create_subscription<EventPacket>(
      "events", rclcpp::SensorDataQoS(),
      [this](EventPacket::ConstSharedPtr msg) { onMsg(msg); });
    frame_thread_ = std::thread([this]() { frameLoop(); });
  }

  // By the time THIS base destructor runs, the derived members the frame thread
  // uses are already destroyed -- so derived classes MUST call stopFrameThread()
  // in their OWN destructor first. This call is a safety net (idempotent).
  ~EventVisionNode() override { stopFrameThread(); }

  // --- event_camera_codecs::EventProcessor (subscription thread) ---
  void eventCD(uint64_t t, uint16_t ex, uint16_t ey, uint8_t polarity) override
  {
    cd_buf_.emplace_back(
      ex, ey, static_cast<short>(polarity), static_cast<Metavision::timestamp>(t));
  }
  bool eventExtTrigger(uint64_t, uint8_t, uint8_t) override { return true; }
  void finished() override {}
  void rawData(const char *, size_t) override {}

protected:
  // Stop and join the frame thread. Derived destructors MUST call this FIRST so
  // the thread (which calls the derived virtuals) is joined before the derived
  // members it touches are destroyed. Idempotent.
  void stopFrameThread()
  {
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (stopping_) {
        return;
      }
      running_ = false;
      stopping_ = true;
    }
    cv_.notify_all();
    if (frame_thread_.joinable()) {
      frame_thread_.join();
    }
  }

  // Create algorithm(s) + frame generator once the sensor geometry is known.
  virtual void onInit(uint16_t width, uint16_t height) = 0;
  // Subscription thread, NO lock: run the algorithm on this packet (incremental)
  // and store its results in a member; the base then calls stageResults().
  virtual void processEvents(const std::vector<Metavision::EventCD> & events) = 0;
  // Subscription thread, called by the base WHILE HOLDING mutex(), ATOMICALLY
  // with the event + timestamp staging: append your computed results to staging.
  // (Splitting this from the event staging would let the frame thread swap in
  // between, leaving results out of sync with the frame timestamp.)
  virtual void stageResults() = 0;
  // Frame thread, called by the base WHILE HOLDING mutex(): swap your staged
  // results into your frame-thread-local buffers.
  virtual void swapResults() = 0;
  // Frame thread, NO lock: render the frame from the swapped events + results.
  // Return false to skip publishing this tick (e.g. nothing new).
  virtual bool renderFrame(
    const std::vector<Metavision::EventCD> & events, Metavision::timestamp ts,
    cv::Mat & frame) = 0;

  std::mutex & mutex() { return mtx_; }
  double fps() const { return fps_; }
  uint32_t accTimeUs() const { return acc_time_us_; }

private:
  void onMsg(EventPacket::ConstSharedPtr msg)
  {
    if (pub_->get_subscription_count() == 0) {
      return;  // lazy
    }
    auto * decoder = factory_.getInstance(*msg);
    if (!decoder) {
      RCLCPP_WARN_ONCE(get_logger(), "unsupported encoding: %s", msg->encoding.c_str());
      return;
    }
    decoder->setTimeMultiplier(1);  // native usec timestamps for the SDK
    cd_buf_.clear();
    while (decoder->decode(*msg, this)) {
    }
    if (cd_buf_.empty()) {
      return;
    }
    if (!inited_) {
      onInit(static_cast<uint16_t>(msg->width), static_cast<uint16_t>(msg->height));
      inited_ = true;
    }
    processEvents(cd_buf_);  // subclass: run algo (no lock), store results
    {
      std::lock_guard<std::mutex> lock(mtx_);
      stageResults();  // stage results atomically with the events + timestamp
      staging_ev_.insert(staging_ev_.end(), cd_buf_.begin(), cd_buf_.end());
      header_ = msg->header;
      last_ts_ = cd_buf_.back().t;
    }
    n_ev_.fetch_add(cd_buf_.size(), std::memory_order_relaxed);
  }

  void frameLoop()
  {
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / fps_));
    auto next = std::chrono::steady_clock::now() + period;
    std::vector<Metavision::EventCD> work;
    while (true) {
      std_msgs::msg::Header header;
      Metavision::timestamp ts = -1;
      work.clear();
      {
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait_until(lock, next, [this]() { return !running_; });
        if (!running_) {
          break;
        }
        next += period;
        const auto now = std::chrono::steady_clock::now();
        if (next < now) {
          next = now + period;  // fell behind: resync, no burst
        }
        std::swap(staging_ev_, work);
        header = header_;
        ts = last_ts_;
        swapResults();  // subclass swaps its results under this same lock
      }
      if (!inited_ || ts < 0) {
        continue;
      }
      const auto t0 = std::chrono::steady_clock::now();
      if (!renderFrame(work, ts, frame_)) {
        continue;
      }
      const auto t1 = std::chrono::steady_clock::now();
      auto img = cv_bridge::CvImage(header, "bgr8", frame_).toImageMsg();
      pub_->publish(*img);
      const auto t2 = std::chrono::steady_clock::now();
      if (debug_timing_) {
        reportTiming(ms(t0, t1), ms(t1, t2));
      }
    }
  }

  static double ms(std::chrono::steady_clock::time_point a, std::chrono::steady_clock::time_point b)
  {
    return std::chrono::duration<double, std::milli>(b - a).count();
  }

  void reportTiming(double render_ms, double pub_ms)
  {
    t_render_ += render_ms;
    t_pub_ += pub_ms;
    ++n_frame_;
    const auto now = std::chrono::steady_clock::now();
    const double sec = std::chrono::duration<double>(now - last_report_).count();
    if (sec < 1.0) {
      return;
    }
    const double mev = n_ev_.exchange(0, std::memory_order_relaxed) / sec / 1e6;
    RCLCPP_INFO(
      get_logger(), "timing: %.0f frame/s %.2f Mev/s | per-frame render=%.2f publish=%.2f ms",
      n_frame_ / sec, mev, n_frame_ ? t_render_ / n_frame_ : 0.0,
      n_frame_ ? t_pub_ / n_frame_ : 0.0);
    t_render_ = t_pub_ = 0.0;
    n_frame_ = 0;
    last_report_ = now;
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<EventPacket>::SharedPtr sub_;
  event_camera_codecs::DecoderFactory<EventPacket, EventVisionNode> factory_;

  std::mutex mtx_;
  std::condition_variable cv_;
  bool running_{true};
  bool stopping_{false};
  std::thread frame_thread_;

  std::vector<Metavision::EventCD> cd_buf_;      // subscription-thread decode buffer
  std::vector<Metavision::EventCD> staging_ev_;  // shared (mtx_)
  std_msgs::msg::Header header_;                 // shared (mtx_)
  Metavision::timestamp last_ts_{-1};            // shared (mtx_)
  cv::Mat frame_;                                // frame-thread only
  bool inited_{false};

  double fps_{30.0};
  uint32_t acc_time_us_{10000};
  bool debug_timing_{false};
  std::atomic<uint64_t> n_ev_{0};
  std::chrono::steady_clock::time_point last_report_{std::chrono::steady_clock::now()};
  double t_render_{0}, t_pub_{0};
  uint64_t n_frame_{0};
};

}  // namespace evk4_sdk_advanced

#endif  // EVK4_SDK_ADVANCED__EVENT_VISION_NODE_HPP_

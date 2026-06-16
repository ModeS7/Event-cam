// Opt-in advanced layer: Metavision SDK (Pro) sparse optical flow on the EVK4
// event stream, rendered to a ROS image.
//
// Pipeline -- the SDK's metavision_sparse_optical_flow sample, adapted to ROS
// and split across two threads for real-time on constrained hardware:
//
//   <camera>/events (EVT3 EventPacket)
//     -- subscription thread (cheap, per packet, never falls behind) --
//     -> decode to std::vector<Metavision::EventCD>     (event_camera_codecs)
//     -> SparseOpticalFlowAlgorithm::process_events      (INCREMENTAL: fed the
//          small per-packet chunk -- the algorithm is streaming and goes
//          super-linear if handed large batches, so we never batch it)
//     -> stage events + flow under a microsecond lock
//     -- frame thread, paced to wall-clock fps --
//     -> swap out the staged events (microsecond lock), then OFF the lock:
//          OnDemandFrameGenerationAlgorithm (accumulate) -> generate ONE frame
//          -> overlay flow arrows -> publish <camera>/flow_image
//
// The lock guards only the buffer swap, never the heavy render -- so event
// ingestion never stalls (no dropped packets, full quality) while the frame
// thread does the expensive copy+publish. Generating on demand on a wall-clock
// timer (not PeriodicFrameGenerationAlgorithm's event-time schedule) avoids the
// catch-up frame burst that otherwise spikes latency. No resolution or quality
// is sacrificed.
//
// The driver owns the camera, so the driver params (ERC cap, biases) govern the
// event rate -- this node only consumes events, over the SDK's
// camera-independent process_events API.

#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_codecs/event_packet.h>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/core/algorithms/on_demand_frame_generation_algorithm.h>
#include <metavision/sdk/cv/algorithms/sparse_flow_frame_generator_algorithm.h>
#include <metavision/sdk/cv/algorithms/sparse_optical_flow_algorithm.h>
#include <metavision/sdk/cv/events/event_optical_flow.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace evk4_sdk_advanced
{
using event_camera_codecs::EventPacket;

class OpticalFlow : public rclcpp::Node, public event_camera_codecs::EventProcessor
{
public:
  explicit OpticalFlow(const rclcpp::NodeOptions & options)
  : Node("optical_flow", options)
  {
    fps_ = declare_parameter("fps", 30.0);
    acc_time_us_ = static_cast<uint32_t>(declare_parameter("accumulation_time_us", 10000));
    debug_timing_ = declare_parameter("debug_timing", false);
    pub_ = create_publisher<sensor_msgs::msg::Image>("flow_image", rclcpp::QoS(1));
    // The driver publishes best-effort; a reliable subscriber would not match.
    sub_ = create_subscription<EventPacket>(
      "events", rclcpp::SensorDataQoS(),
      [this](EventPacket::ConstSharedPtr msg) { onMsg(msg); });
    frame_thread_ = std::thread([this]() { frameLoop(); });
    RCLCPP_INFO(
      get_logger(), "sparse optical flow: %.0f fps, %u us accumulation window",
      fps_, acc_time_us_);
  }

  ~OpticalFlow() override
  {
    {
      std::lock_guard<std::mutex> lock(mtx_);
      running_ = false;
    }
    cv_.notify_all();
    if (frame_thread_.joinable()) {
      frame_thread_.join();
    }
  }

  // --- event_camera_codecs::EventProcessor: one call per decoded event ---
  // Runs on the subscription thread; appends to that thread's private buffer.
  void eventCD(uint64_t t, uint16_t ex, uint16_t ey, uint8_t polarity) override
  {
    cd_buf_.emplace_back(
      ex, ey, static_cast<short>(polarity), static_cast<Metavision::timestamp>(t));
  }
  bool eventExtTrigger(uint64_t, uint8_t, uint8_t) override { return true; }
  void finished() override {}
  void rawData(const char *, size_t) override {}

private:
  // Subscription thread: decode, run flow incrementally, stage the results.
  // Holds the lock only for the cheap append, so it never falls behind.
  void onMsg(EventPacket::ConstSharedPtr msg)
  {
    if (pub_->get_subscription_count() == 0) {
      return;  // lazy: nothing to render for nobody
    }
    auto * decoder = factory_.getInstance(*msg);
    if (!decoder) {
      RCLCPP_WARN_ONCE(get_logger(), "unsupported encoding: %s", msg->encoding.c_str());
      return;
    }
    // Native usec timestamps (the SDK works in usec); without this the codec
    // emits nanoseconds and the flow time constants would be off by 1000x.
    decoder->setTimeMultiplier(1);

    cd_buf_.clear();
    while (decoder->decode(*msg, this)) {
    }
    if (cd_buf_.empty()) {
      return;
    }
    const auto w = static_cast<uint16_t>(msg->width);
    const auto h = static_cast<uint16_t>(msg->height);
    if (!flow_algo_) {
      flow_algo_ = std::make_unique<Metavision::SparseOpticalFlowAlgorithm>(
        w, h, Metavision::SparseOpticalFlowConfig::Preset::FastObjects);
    }

    // Flow runs here (flow_algo_ is touched only on this thread), fed the small
    // per-packet chunk -> stays linear.
    const auto fa = std::chrono::steady_clock::now();
    flow_out_.clear();
    flow_algo_->process_events(cd_buf_.begin(), cd_buf_.end(), std::back_inserter(flow_out_));
    const auto fb = std::chrono::steady_clock::now();

    {
      std::lock_guard<std::mutex> lock(mtx_);  // cheap: just appends + a few scalars
      width_ = w;
      height_ = h;
      staging_ev_.insert(staging_ev_.end(), cd_buf_.begin(), cd_buf_.end());
      staging_fl_.insert(staging_fl_.end(), flow_out_.begin(), flow_out_.end());
      last_ts_ = cd_buf_.back().t;
      header_ = msg->header;
    }
    n_ev_.fetch_add(cd_buf_.size(), std::memory_order_relaxed);
    if (debug_timing_) {
      t_flow_us_.fetch_add(static_cast<uint64_t>(ms(fa, fb) * 1000.0), std::memory_order_relaxed);
    }
  }

  // Frame thread: wall-clock paced. Swaps out staged events under a microsecond
  // lock, then does ALL heavy work (feed/generate/overlay/copy/publish) off the
  // lock so it never blocks ingestion.
  void frameLoop()
  {
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / fps_));
    auto next = std::chrono::steady_clock::now() + period;
    while (true) {
      std_msgs::msg::Header header;
      Metavision::timestamp ts = -1;
      uint16_t w = 0, h = 0;
      work_ev_.clear();
      work_fl_.clear();
      {
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait_until(lock, next, [this]() { return !running_; });
        if (!running_) {
          break;
        }
        next += period;
        const auto now = std::chrono::steady_clock::now();
        if (next < now) {
          next = now + period;  // fell behind: resync, do not burst to catch up
        }
        std::swap(staging_ev_, work_ev_);
        std::swap(staging_fl_, work_fl_);
        ts = last_ts_;
        header = header_;
        w = width_;
        h = height_;
      }
      if (w == 0 || ts < 0 || work_ev_.empty()) {
        continue;  // no geometry yet, or quiet scene: hold the last frame
      }
      if (!frame_gen_) {
        frame_gen_ = std::make_unique<Metavision::OnDemandFrameGenerationAlgorithm>(
          w, h, acc_time_us_);
        flow_frame_gen_ = std::make_unique<Metavision::SparseFlowFrameGeneratorAlgorithm>();
        RCLCPP_INFO(get_logger(), "pipeline initialized for %ux%u", w, h);
      }

      const auto t0 = std::chrono::steady_clock::now();
      frame_gen_->process_events(work_ev_.begin(), work_ev_.end());  // cheap accumulate
      flow_buf_.insert(flow_buf_.end(), work_fl_.begin(), work_fl_.end());
      frame_gen_->generate(ts, frame_);                              // render event image
      const auto t1 = std::chrono::steady_clock::now();
      overlayFlow(ts, frame_);                                       // draw flow arrows
      const auto t2 = std::chrono::steady_clock::now();
      auto img = cv_bridge::CvImage(header, "bgr8", frame_).toImageMsg();
      const auto t3 = std::chrono::steady_clock::now();
      pub_->publish(*img);
      const auto t4 = std::chrono::steady_clock::now();

      if (debug_timing_) {
        t_gen_ += ms(t0, t1);
        t_overlay_ += ms(t1, t2);
        t_copy_ += ms(t2, t3);
        t_pub_ += ms(t3, t4);
        ++n_frame_;
        reportTiming();
      }
    }
  }

  // Overlay this window's flow arrows onto the event image (mirrors the sample).
  // Frame thread only (flow_buf_ / flow_frame_gen_ are not shared).
  void overlayFlow(Metavision::timestamp ts, cv::Mat & frame)
  {
    if (flow_buf_.empty()) {
      return;
    }
    const Metavision::timestamp ts_begin = ts - static_cast<Metavision::timestamp>(acc_time_us_);
    auto it_begin = std::lower_bound(
      flow_buf_.begin(), flow_buf_.end(), ts_begin,
      [](const Metavision::EventOpticalFlow & ev, Metavision::timestamp t) { return ev.t < t; });
    auto it_end = std::upper_bound(
      flow_buf_.begin(), flow_buf_.end(), ts,
      [](Metavision::timestamp t, const Metavision::EventOpticalFlow & ev) { return t < ev.t; });
    if (it_begin != it_end) {
      flow_frame_gen_->add_flow_for_frame_update(it_begin, it_end);
    }
    flow_frame_gen_->update_frame_with_flow(frame);
    flow_frame_gen_->clear_ids();
    // Drop flow events older than the next frame's window.
    const Metavision::timestamp ts_remove =
      ts - static_cast<Metavision::timestamp>(acc_time_us_) +
      static_cast<Metavision::timestamp>(1e6 / fps_);
    auto it_remove = std::lower_bound(
      flow_buf_.begin(), flow_buf_.end(), ts_remove,
      [](const Metavision::EventOpticalFlow & ev, Metavision::timestamp t) { return ev.t < t; });
    flow_buf_.erase(flow_buf_.begin(), it_remove);
  }

  static double ms(std::chrono::steady_clock::time_point a, std::chrono::steady_clock::time_point b)
  {
    return std::chrono::duration<double, std::milli>(b - a).count();
  }

  void reportTiming()
  {
    const auto now = std::chrono::steady_clock::now();
    const double sec = std::chrono::duration<double>(now - last_report_).count();
    if (sec < 1.0) {
      return;
    }
    const double mev = n_ev_.exchange(0, std::memory_order_relaxed) / sec / 1e6;
    // flow ms spent per wall second on the sub thread; < 1000 => it keeps up.
    const double flow_ms_per_s = (t_flow_us_.exchange(0, std::memory_order_relaxed) / 1000.0) / sec;
    RCLCPP_INFO(
      get_logger(),
      "timing: %.0f frame/s %.2f Mev/s | flow=%.0f ms/s (sub) | per-frame generate=%.2f "
      "overlay=%.2f copy=%.2f publish=%.2f ms",
      n_frame_ / sec, mev, flow_ms_per_s, n_frame_ ? t_gen_ / n_frame_ : 0.0,
      n_frame_ ? t_overlay_ / n_frame_ : 0.0, n_frame_ ? t_copy_ / n_frame_ : 0.0,
      n_frame_ ? t_pub_ / n_frame_ : 0.0);
    t_gen_ = t_overlay_ = t_copy_ = t_pub_ = 0.0;
    n_frame_ = 0;
    last_report_ = now;
  }

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<EventPacket>::SharedPtr sub_;
  event_camera_codecs::DecoderFactory<EventPacket, OpticalFlow> factory_;

  // Thread coordination
  std::mutex mtx_;
  std::condition_variable cv_;
  bool running_{true};
  std::thread frame_thread_;

  // Subscription-thread only
  std::vector<Metavision::EventCD> cd_buf_;
  std::vector<Metavision::EventOpticalFlow> flow_out_;
  std::unique_ptr<Metavision::SparseOpticalFlowAlgorithm> flow_algo_;

  // Shared, guarded by mtx_ (held only for the swap/append)
  std::vector<Metavision::EventCD> staging_ev_;
  std::vector<Metavision::EventOpticalFlow> staging_fl_;
  Metavision::timestamp last_ts_{-1};
  std_msgs::msg::Header header_;
  uint16_t width_{0}, height_{0};

  // Frame-thread only
  std::vector<Metavision::EventCD> work_ev_;
  std::vector<Metavision::EventOpticalFlow> work_fl_;
  std::unique_ptr<Metavision::OnDemandFrameGenerationAlgorithm> frame_gen_;
  std::unique_ptr<Metavision::SparseFlowFrameGeneratorAlgorithm> flow_frame_gen_;
  std::vector<Metavision::EventOpticalFlow> flow_buf_;
  cv::Mat frame_;

  // Params
  double fps_{30.0};
  uint32_t acc_time_us_{10000};

  // Optional per-second stage timing (debug_timing param).
  bool debug_timing_{false};
  std::atomic<uint64_t> n_ev_{0};
  std::atomic<uint64_t> t_flow_us_{0};
  std::chrono::steady_clock::time_point last_report_{std::chrono::steady_clock::now()};
  double t_gen_{0}, t_overlay_{0}, t_copy_{0}, t_pub_{0};
  uint64_t n_frame_{0};
};

}  // namespace evk4_sdk_advanced

RCLCPP_COMPONENTS_REGISTER_NODE(evk4_sdk_advanced::OpticalFlow)

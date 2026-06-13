// Example subscriber: decode EventPacket messages and log event statistics.
// C++ twin of the Python example in evk4_examples.
//
// Standalone (against the topics of evk4.launch.py):
//   ros2 run evk4_examples_cpp event_rate
//
// Or load it into the running camera container for the zero-copy
// intra-process path (see docs/usage.md):
//   ros2 component load /event_camera_container evk4_examples_cpp evk4_examples_cpp::EventRate -e use_intra_process_comms:=true

#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_codecs/event_packet.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace evk4_examples_cpp
{
using event_camera_codecs::EventPacket;

// Receives one callback per decoded event; just counts polarities.
class EventCounter : public event_camera_codecs::EventProcessor
{
public:
  void eventCD(uint64_t, uint16_t, uint16_t, uint8_t polarity) override
  {
    polarity ? onEvents_++ : offEvents_++;
  }
  bool eventExtTrigger(uint64_t, uint8_t, uint8_t) override { return (true); }
  void finished() override {}
  void rawData(const char *, size_t) override {}

  uint64_t onEvents() const { return (onEvents_); }
  uint64_t offEvents() const { return (offEvents_); }
  void reset() { onEvents_ = offEvents_ = 0; }

private:
  uint64_t onEvents_{0};
  uint64_t offEvents_{0};
};

class EventRate : public rclcpp::Node
{
public:
  explicit EventRate(const rclcpp::NodeOptions & options)
  : Node("event_rate_cpp", options)
  {
    interval_ = declare_parameter("print_interval", 2.0);  // [s]
    // The driver publishes best-effort: a reliable subscriber would not match.
    subscription_ = create_subscription<EventPacket>(
      "event_camera/events", rclcpp::SensorDataQoS(),
      [this](EventPacket::ConstSharedPtr msg) { onMsg(msg); });
    timer_ = create_wall_timer(
      std::chrono::duration<double>(interval_), [this]() { printStats(); });
    RCLCPP_INFO(
      get_logger(), "listening on %s (stats every %.1f s)",
      subscription_->get_topic_name(), interval_);
  }

private:
  void onMsg(EventPacket::ConstSharedPtr msg)
  {
    msgs_++;
    auto * decoder = decoderFactory_.getInstance(*msg);
    if (!decoder) {
      RCLCPP_WARN_ONCE(get_logger(), "unsupported encoding: %s", msg->encoding.c_str());
      return;
    }
    // decode() must be called until it returns false (all bytes consumed)
    while (decoder->decode(*msg, &counter_)) {
    }
  }

  void printStats()
  {
    const uint64_t total = counter_.onEvents() + counter_.offEvents();
    if (msgs_ == 0) {
      RCLCPP_WARN(
        get_logger(),
        "no messages on %s - is the camera running and the scene changing? "
        "(check: ros2 topic hz <events topic>)",
        subscription_->get_topic_name());
    } else {
      const double onPct = total ? 100.0 * counter_.onEvents() / total : 0.0;
      RCLCPP_INFO(
        get_logger(), "%7.3f Mev/s  (ON %4.1f%%)  %6.1f msgs/s",
        total / interval_ / 1e6, onPct, msgs_ / interval_);
    }
    counter_.reset();
    msgs_ = 0;
  }

  EventCounter counter_;
  event_camera_codecs::DecoderFactory<EventPacket, EventCounter> decoderFactory_;
  rclcpp::Subscription<EventPacket>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  double interval_{2.0};
  uint64_t msgs_{0};
};

}  // namespace evk4_examples_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(evk4_examples_cpp::EventRate)

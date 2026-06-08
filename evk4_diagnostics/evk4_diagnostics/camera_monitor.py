"""Watchdog: publish EVK4 event-stream health on /diagnostics.

Distinguishes the three states that look identical from the outside:

    OK     events are flowing (reports msgs/s, MB/s, Mev/s)
    WARN   driver node alive but silent for > warn_timeout
           (legitimate for a static scene, suspicious otherwise)
    ERROR  driver node missing from the ROS graph

Usage (with the default evk4.launch.py topics):

    ros2 run evk4_diagnostics camera_monitor

View with `ros2 topic echo /diagnostics` or `rqt_runtime_monitor`.
For a non-default camera name set the parameter:

    ros2 run evk4_diagnostics camera_monitor --ros-args -p camera_node:=my_camera
"""

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticTask, Updater
from event_camera_msgs.msg import EventPacket
from event_camera_py import Decoder
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class StreamHealthTask(DiagnosticTask):
    """Adapter: lets the Updater poll the monitor node for status."""

    def __init__(self, monitor):
        super().__init__('event stream')
        self._monitor = monitor

    def run(self, stat):
        return self._monitor.fill_status(stat)


class CameraMonitor(Node):
    """Track event stream rates and publish them as diagnostics."""

    def __init__(self):
        super().__init__('camera_monitor')
        self.declare_parameter('camera_node', 'event_camera')
        self.declare_parameter('warn_timeout', 5.0)  # [s]
        self._camera_node = self.get_parameter('camera_node').value
        self._warn_timeout = self.get_parameter('warn_timeout').value
        self._decoder = Decoder()
        self._msgs = 0
        self._bytes = 0
        self._events = 0
        self._last_msg_time = None
        self._window_start = self._now()
        # The driver publishes best-effort; a reliable subscriber would
        # not match.
        self._subscription = self.create_subscription(
            EventPacket, f'{self._camera_node}/events', self._on_msg,
            qos_profile_sensor_data)
        # Publishes /diagnostics once per second by default
        # (diagnostic_updater.period parameter).
        self._updater = Updater(self)
        self._updater.setHardwareID(f'evk4/{self._camera_node}')
        self._updater.add(StreamHealthTask(self))
        self.get_logger().info(
            f'monitoring {self._subscription.topic_name} '
            f"(driver node '{self._camera_node}')")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_msg(self, msg):
        self._msgs += 1
        self._bytes += len(msg.events)
        self._decoder.decode(msg)
        self._events += self._decoder.get_cd_events().size
        self._last_msg_time = self._now()

    def fill_status(self, stat):
        now = self._now()
        dt = max(now - self._window_start, 1e-6)
        msgs_s = self._msgs / dt
        mb_s = self._bytes / dt / 1e6
        mev_s = self._events / dt / 1e6
        self._msgs = self._bytes = self._events = 0
        self._window_start = now

        driver_alive = self._camera_node in self.get_node_names()
        silent_for = None if self._last_msg_time is None \
            else now - self._last_msg_time

        if not driver_alive:
            stat.summary(
                DiagnosticStatus.ERROR,
                f"driver node '{self._camera_node}' not in ROS graph")
        elif silent_for is None:
            stat.summary(DiagnosticStatus.WARN, 'no messages received yet')
        elif silent_for > self._warn_timeout:
            stat.summary(
                DiagnosticStatus.WARN,
                f'no events for {silent_for:.1f} s '
                '(driver alive - static scene or stalled stream)')
        else:
            stat.summary(DiagnosticStatus.OK, 'streaming')
        stat.add('msgs/s', f'{msgs_s:.1f}')
        stat.add('MB/s', f'{mb_s:.2f}')
        stat.add('Mev/s', f'{mev_s:.3f}')
        return stat


def main(args=None):
    """Run the camera monitor until interrupted."""
    rclpy.init(args=args)
    node = CameraMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

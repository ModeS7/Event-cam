# Copyright 2026 Modestas
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Example subscriber: decode EventPacket messages and log event statistics.

Works out of the box against the topics published by
`ros2 launch evk4_bringup evk4.launch.py`:

    ros2 run evk4_examples event_rate

For a non-default camera name, remap the topic:

    ros2 run evk4_examples event_rate --ros-args \
        -r event_camera/events:=/my_camera/events
"""

import rclpy
from event_camera_msgs.msg import EventPacket
from event_camera_py import Decoder
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class EventRateNode(Node):
    """Decode EventPacket messages and periodically log event rates."""

    def __init__(self):
        super().__init__('event_rate')
        self.declare_parameter('print_interval', 2.0)  # [s]
        self._interval = self.get_parameter('print_interval').value
        self._decoder = Decoder()
        self._msgs = 0
        self._on_events = 0
        self._off_events = 0
        # The driver publishes best-effort: a default (reliable) subscriber
        # would not match and receive nothing.
        self._subscription = self.create_subscription(
            EventPacket, 'event_camera/events', self._on_msg,
            qos_profile_sensor_data)
        self.create_timer(self._interval, self._print_stats)
        self.get_logger().info(
            f'listening on {self._subscription.topic_name} '
            f'(stats every {self._interval:.1f} s)')

    def _on_msg(self, msg):
        self._msgs += 1
        self._decoder.decode(msg)
        # Structured numpy array with fields x, y, p (polarity), t [us].
        cd_events = self._decoder.get_cd_events()
        if cd_events.size:
            on = int(cd_events['p'].sum())
            self._on_events += on
            self._off_events += cd_events.size - on

    def _print_stats(self):
        total = self._on_events + self._off_events
        if self._msgs == 0:
            self.get_logger().warning(
                f'no messages on {self._subscription.topic_name} - is the '
                'camera running and the scene changing? '
                '(check: ros2 topic hz <events topic>)')
        else:
            on_pct = 100.0 * self._on_events / total if total else 0.0
            self.get_logger().info(
                f'{total / self._interval / 1e6:7.3f} Mev/s  '
                f'(ON {on_pct:4.1f}%)  '
                f'{self._msgs / self._interval:6.1f} msgs/s')
        self._msgs = 0
        self._on_events = 0
        self._off_events = 0


def main(args=None):
    rclpy.init(args=args)
    node = EventRateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

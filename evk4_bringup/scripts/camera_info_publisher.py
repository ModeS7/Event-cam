#!/usr/bin/env python3
"""Publish CameraInfo matched to image_raw, so image_proc can rectify.

The metavision_driver / renderer publish no CameraInfo. This node loads a
standard ROS camera_info YAML (the format `camera_calibration` writes) and
republishes it as sensor_msgs/CameraInfo with the SAME header as each
incoming image_raw — same timestamp so image_proc can pair them, same
frame_id so the camera_info matches the image's frame regardless of what the
driver stamped.

    ros2 run evk4_bringup camera_info_publisher.py \
        --ros-args -p calibration_url:=/path/to/event_camera.yaml \
        -r image_raw:=/event_camera/image_raw \
        -r camera_info:=/event_camera/camera_info
"""

import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


def _load_camera_info(path):
    """Build a CameraInfo from a standard ROS camera_info YAML file."""
    with open(path) as f:
        calib = yaml.safe_load(f)
    info = CameraInfo()
    info.width = calib['image_width']
    info.height = calib['image_height']
    info.distortion_model = calib.get('distortion_model', 'plumb_bob')
    info.d = list(calib['distortion_coefficients']['data'])
    info.k = list(calib['camera_matrix']['data'])
    info.r = list(calib['rectification_matrix']['data'])
    info.p = list(calib['projection_matrix']['data'])
    return info


class CameraInfoPublisher(Node):
    """Echo a fixed CameraInfo on every image_raw, with the image's header."""

    def __init__(self):
        super().__init__('camera_info_publisher')
        url = self.declare_parameter('calibration_url', '').value
        if not url:
            raise RuntimeError('calibration_url parameter is required')
        self._info = _load_camera_info(url)
        self._pub = self.create_publisher(CameraInfo, 'camera_info', 10)
        # image_transport publishes image_raw with default (reliable) QoS.
        self._sub = self.create_subscription(Image, 'image_raw', self._on_image, 10)
        self.get_logger().info(
            f'publishing camera_info from {url} '
            f'({self._info.width}x{self._info.height})')

    def _on_image(self, image):
        self._info.header = image.header   # same stamp and frame_id
        self._pub.publish(self._info)


def main(args=None):
    """Run the camera_info publisher until interrupted."""
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

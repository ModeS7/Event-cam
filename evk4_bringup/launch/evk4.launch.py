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

"""Bring up the Prophesee EVK4: metavision_driver plus optional renderer.

Topic contract (with default arguments):

    /event_camera/events     event_camera_msgs/msg/EventPacket   (always)
    /event_camera/image_raw  sensor_msgs/msg/Image               (viz:=true)
"""

import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _require(package, apt_package):
    """Fail loudly at launch time if a wrapped package is missing."""
    try:
        get_package_share_directory(package)
    except PackageNotFoundError as exc:
        raise RuntimeError(
            f"required package '{package}' is not installed. "
            f'Install it with: sudo apt install {apt_package}') from exc


def _launch_setup(context, *args, **kwargs):
    camera_name = LaunchConfiguration('camera_name').perform(context)
    serial = LaunchConfiguration('serial').perform(context)
    bias_file = LaunchConfiguration('bias_file').perform(context)
    viz = LaunchConfiguration('viz').perform(context).lower()
    if viz not in ('true', 'false'):
        raise RuntimeError(f"viz must be 'true' or 'false', got '{viz}'")

    _require('metavision_driver', 'ros-jazzy-metavision-driver')
    params_file = os.path.join(
        get_package_share_directory('evk4_bringup'), 'config', 'evk4_params.yaml')

    nodes = [
        Node(
            package='metavision_driver',
            executable='driver_node',
            name=camera_name,
            output='screen',
            parameters=[
                params_file,
                {
                    'serial': serial,
                    'bias_file': bias_file,
                    'frame_id': camera_name,
                },
            ],
        ),
    ]
    if viz == 'true':
        _require('event_camera_renderer', 'ros-jazzy-event-camera-renderer')
        nodes.append(
            Node(
                package='event_camera_renderer',
                executable='renderer_node',
                name='renderer',
                output='screen',
                remappings=[
                    ('~/events', f'/{camera_name}/events'),
                    ('~/image_raw', f'/{camera_name}/image_raw'),
                ],
            ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name', default_value='event_camera',
            description='Node name and topic namespace for the camera.'),
        DeclareLaunchArgument(
            'serial', default_value='',
            description='Camera serial number (empty: first camera found).'),
        DeclareLaunchArgument(
            'bias_file', default_value='',
            description='Path to a .bias file (empty: sensor defaults).'),
        DeclareLaunchArgument(
            'viz', default_value='true',
            description='Also run event_camera_renderer, publishing '
                        '<camera_name>/image_raw.'),
        OpaqueFunction(function=_launch_setup),
    ])

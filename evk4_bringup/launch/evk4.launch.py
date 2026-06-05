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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


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

    # Driver and renderer are composed into ONE container process with
    # intra-process communication: the high-rate event stream passes
    # between them as pointers instead of being serialized and copied
    # through the middleware. Subscribers in other processes (your nodes,
    # rosbag2, rqt) still receive normal DDS copies.
    components = [
        ComposableNode(
            package='metavision_driver',
            plugin='metavision_driver::DriverROS2',
            name=camera_name,
            # NOTE: driver 3.0.0 has no frame_id parameter (master-only);
            # message headers carry the last 4 digits of the serial number.
            parameters=[
                params_file,
                {
                    'serial': serial,
                    'bias_file': bias_file,
                },
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]
    if viz == 'true':
        _require('event_camera_renderer', 'ros-jazzy-event-camera-renderer')
        components.append(
            ComposableNode(
                package='event_camera_renderer',
                plugin='event_camera_renderer::Renderer',
                name='renderer',
                remappings=[
                    ('~/events', f'/{camera_name}/events'),
                    ('~/image_raw', f'/{camera_name}/image_raw'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ))
    return [
        ComposableNodeContainer(
            name=f'{camera_name}_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_isolated',
            composable_node_descriptions=components,
            output='screen',
        ),
    ]


def generate_launch_description():
    """Declare launch arguments and defer node creation to _launch_setup."""
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

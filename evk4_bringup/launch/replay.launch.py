"""Replay an event bag through the renderer -- no camera required.

This is the renderer side of evk4.launch.py with the driver removed: it runs
event_camera_renderer (and, optionally, camera_info + rectification) subscribed
to <camera_name>/events, and waits. Feed it from a bag in another terminal:

    ros2 launch evk4_bringup replay.launch.py
    ros2 bag play <bag>          # republishes /event_camera/events
    ros2 run rqt_image_view rqt_image_view /event_camera/image_raw

Use it to try the pipeline with no hardware (see docs/usage.md, "No camera?").
The camera_name argument must match the bag's topic namespace (the default,
event_camera, matches a bag recorded from the default launch).

Topics published (with default arguments):

    /event_camera/image_raw    sensor_msgs/msg/Image
    /event_camera/camera_info  sensor_msgs/msg/CameraInfo   (calibration_url set)
    /event_camera/image_rect   sensor_msgs/msg/Image        (rectify:=true)
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


def _require(package):
    """Fail loudly at launch time if a required package is missing."""
    try:
        get_package_share_directory(package)
    except PackageNotFoundError as exc:
        if package.startswith('evk4_'):
            hint = ('build this repo: cd ~/ros2_ws && colcon build '
                    '--symlink-install && source install/setup.bash')
        else:
            distro = os.environ.get('ROS_DISTRO', '<distro>')
            apt_name = f"ros-{distro}-{package.replace('_', '-')}"
            hint = (f'install it (binary platform: sudo apt install {apt_name}; '
                    'ARM/source platforms: build it for your target)')
        raise RuntimeError(
            f"required package '{package}' is not found. {hint}") from exc


def _launch_setup(context, *args, **kwargs):
    camera_name = LaunchConfiguration('camera_name').perform(context)
    calibration_url = LaunchConfiguration('calibration_url').perform(context)
    rectify = LaunchConfiguration('rectify').perform(context).lower()
    fps = float(LaunchConfiguration('fps').perform(context))
    display_type = LaunchConfiguration('display_type').perform(context)

    _require('event_camera_renderer')
    # The bag supplies <camera_name>/events; render it exactly as the live
    # launch does, so a replayed stream looks identical to a live one.
    components = [
        ComposableNode(
            package='event_camera_renderer',
            plugin='event_camera_renderer::Renderer',
            name='renderer',
            namespace=camera_name,
            # use_sim_time is essential for replay: the renderer times its
            # frames off the clock's now(), but the bag's events carry old
            # recorded stamps. On wall-clock time the target frame times sit
            # ahead of the replayed events and nothing is ever emitted. Running
            # on sim time (with `ros2 bag play --clock`) aligns the two.
            parameters=[{'fps': fps, 'display_type': display_type,
                         'use_sim_time': True}],
            remappings=[
                ('~/events', f'/{camera_name}/events'),
                ('~/image_raw', f'/{camera_name}/image_raw'),
            ],
            # No intra-process here: the events arrive over DDS from a separate
            # `ros2 bag play` process, so there is no pointer to pass.
        ),
    ]

    if calibration_url:
        components.append(
            ComposableNode(
                package='evk4_bringup',
                plugin='evk4_bringup::CameraInfoPublisher',
                name='camera_info_publisher',
                namespace=camera_name,
                parameters=[{'calibration_url': calibration_url,
                             'use_sim_time': True}],
                remappings=[('image_raw', f'/{camera_name}/image_raw'),
                            ('camera_info', f'/{camera_name}/camera_info')],
            ))

    if rectify == 'true':
        if not calibration_url:
            raise RuntimeError('rectify:=true needs calibration_url')
        _require('image_proc')
        components.append(
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify',
                namespace=camera_name,
                parameters=[{'interpolation': 0, 'use_sim_time': True}],
                remappings=[('image', f'/{camera_name}/image_raw'),
                            ('camera_info', f'/{camera_name}/camera_info'),
                            ('image_rect', f'/{camera_name}/image_rect')],
            ))

    return [
        ComposableNodeContainer(
            name=f'{camera_name}_replay_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_isolated',
            composable_node_descriptions=components,
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name', default_value='event_camera',
            description='Topic namespace; must match the bag (default '
                        'event_camera matches a bag from the default launch).'),
        DeclareLaunchArgument(
            'fps', default_value='25.0',
            description='Renderer frame rate in Hz.'),
        DeclareLaunchArgument(
            'display_type', default_value='time_slice',
            description='Renderer mode: time_slice or sharp.'),
        DeclareLaunchArgument(
            'calibration_url', default_value='',
            description='Path to a camera_info YAML; when set, publish '
                        '<camera_name>/camera_info for rectification.'),
        DeclareLaunchArgument(
            'rectify', default_value='false',
            description='Also publish <camera_name>/image_rect (undistorted); '
                        'needs calibration_url.'),
        OpaqueFunction(function=_launch_setup),
    ])

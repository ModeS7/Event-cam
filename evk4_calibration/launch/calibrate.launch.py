"""One-command calibration session: camera + calibrator + viewer.

Composes the standard camera bringup (evk4.launch.py), the headless
calibrator, and rqt_image_view on the calibrator's overlay. When the
calibrator finishes (it writes the camera_info YAML and exits), the whole
session shuts down by itself.

    ros2 launch evk4_calibration calibrate.launch.py \\
        params_file:=$HOME/my_params.yaml

The YAML is written in the directory this command was started from.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('evk4_bringup'),
            'launch', 'evk4.launch.py')),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'display_type': LaunchConfiguration('display_type'),
        }.items(),
    )
    calibrator = Node(
        package='evk4_calibration',
        executable='calibrate',
        name='calibrate',
        output='screen',
        parameters=[{
            'grid_size': LaunchConfiguration('grid_size'),
            'output': LaunchConfiguration('output'),
        }],
        remappings=[('image_raw', '/event_camera/image_raw')],
    )
    viewer = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=['/calibrate/overlay'],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Driver params YAML (your ~/my_params.yaml from '
                        'docs/tuning.md, including startup biases).'),
        DeclareLaunchArgument(
            'display_type', default_value='time_slice',
            description='Renderer mode during calibration (sharp lags on '
                        'quiet scenes; see tuning.md).'),
        DeclareLaunchArgument(
            'grid_size', default_value='5x17',
            description='Circles per row x rows (docs/circle_grid.html).'),
        DeclareLaunchArgument(
            'output', default_value='event_camera.yaml',
            description='Output camera_info YAML path.'),
        camera,
        calibrator,
        viewer,
        # The calibrator exits when the calibration is written; end the
        # whole session with it.
        RegisterEventHandler(OnProcessExit(
            target_action=calibrator,
            on_exit=[Shutdown(reason='calibration complete')])),
    ])

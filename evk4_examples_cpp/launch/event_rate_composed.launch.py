"""Launch the EVK4 camera with event_rate_cpp composed into its container.

One command instead of evk4.launch.py + `ros2 component load ...`:

    ros2 launch evk4_examples_cpp event_rate_composed.launch.py

The example is loaded into /event_camera_container with intra-process
communication, so it receives the event stream by pointer (zero-copy).
Assumes the default camera_name (event_camera).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Start the camera container, then load the event_rate_cpp component."""
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('evk4_bringup'),
                'launch', 'evk4.launch.py')),
        launch_arguments={'viz': LaunchConfiguration('viz')}.items(),
    )
    load_event_rate = LoadComposableNodes(
        target_container='/event_camera_container',
        composable_node_descriptions=[
            ComposableNode(
                package='evk4_examples_cpp',
                plugin='evk4_examples_cpp::EventRate',
                name='event_rate_cpp',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'viz', default_value='true',
            description='Also run the renderer (see evk4.launch.py).'),
        camera,
        load_event_rate,
    ])

"""Standalone test bringup: evk4_driver + renderer composed in one container.

A minimal driver-only launch (no evk4_bringup config) for exercising
evk4_driver directly: it publishes raw EVT3 as EventPacket and the renderer
turns it into image_raw, all intra-process in a single container.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    cam = 'event_camera'
    return LaunchDescription([
        DeclareLaunchArgument('serial', default_value=''),
        DeclareLaunchArgument('afk_enabled', default_value='false'),
        ComposableNodeContainer(
            name=f'{cam}_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_isolated',
            composable_node_descriptions=[
                ComposableNode(
                    package='evk4_driver',
                    plugin='evk4_driver::EVK4Driver',
                    name=cam,
                    parameters=[{
                        'serial': LaunchConfiguration('serial'),
                        'afk_enabled': LaunchConfiguration('afk_enabled'),
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='event_camera_renderer',
                    plugin='event_camera_renderer::Renderer',
                    name='renderer',
                    namespace=cam,
                    parameters=[{'fps': 25.0}],
                    remappings=[
                        ('~/events', f'/{cam}/events'),
                        ('~/image_raw', f'/{cam}/image_raw'),
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen',
        ),
    ])

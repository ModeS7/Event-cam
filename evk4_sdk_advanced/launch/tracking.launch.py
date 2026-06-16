"""EVK4 driver + Metavision SDK object tracking.

    ros2 launch evk4_sdk_advanced tracking.launch.py \
        params_file:=$HOME/my_params.yaml

Publishes /<camera_name>/tracking_image (sensor_msgs/Image) -- event edges with
bounding boxes on tracked moving objects. Same structure as
optical_flow.launch.py: the driver (openeb_vendor) owns the camera in its own
process; the tracking node links the SDK Pro build and runs as a separate
process with the SDK libs on its LD_LIBRARY_PATH (captured at build time) -- no
`source setup_env.sh` needed.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    camera_name = LaunchConfiguration('camera_name').perform(context)
    serial = LaunchConfiguration('serial').perform(context)
    frame_id = LaunchConfiguration('frame_id').perform(context)
    fps = float(LaunchConfiguration('fps').perform(context))
    min_size = int(LaunchConfiguration('min_size').perform(context))
    max_size = int(LaunchConfiguration('max_size').perform(context))
    debug_timing = LaunchConfiguration('debug_timing').perform(context).lower() == 'true'
    params_file = LaunchConfiguration('params_file').perform(context)

    sdk_libdir = ''
    libdir_file = os.path.join(
        get_package_share_directory('evk4_sdk_advanced'), 'sdk_libdir')
    if os.path.exists(libdir_file):
        with open(libdir_file) as handle:
            sdk_libdir = handle.read().strip()
    node_env = {}
    if sdk_libdir:
        node_env['LD_LIBRARY_PATH'] = (
            sdk_libdir + os.pathsep + os.environ.get('LD_LIBRARY_PATH', ''))

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('evk4_bringup'), 'launch', 'evk4.launch.py')),
        launch_arguments={
            'camera_name': camera_name,
            'serial': serial,
            'frame_id': frame_id,
            'viz': 'false',
            'params_file': params_file,
        }.items(),
    )

    tracking = Node(
        package='evk4_sdk_advanced',
        executable='tracking',
        name='tracking',
        namespace=camera_name,
        parameters=[{
            'fps': fps, 'min_size': min_size, 'max_size': max_size,
            'debug_timing': debug_timing}],
        remappings=[
            ('events', f'/{camera_name}/events'),
            ('tracking_image', f'/{camera_name}/tracking_image'),
        ],
        additional_env=node_env,
        output='screen',
    )

    return [driver, tracking]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='event_camera'),
        DeclareLaunchArgument('serial', default_value=''),
        DeclareLaunchArgument('frame_id', default_value='event_camera_optical_frame'),
        DeclareLaunchArgument('fps', default_value='30.0'),
        DeclareLaunchArgument(
            'min_size', default_value='10',
            description='Minimum tracked-object size in pixels.'),
        DeclareLaunchArgument(
            'max_size', default_value='300',
            description='Maximum tracked-object size in pixels.'),
        DeclareLaunchArgument('debug_timing', default_value='false'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Driver params YAML (use your ~/my_params.yaml).'),
        OpaqueFunction(function=_launch_setup),
    ])

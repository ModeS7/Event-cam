"""EVK4 driver + Metavision SDK sparse optical flow.

    ros2 launch evk4_sdk_advanced optical_flow.launch.py \
        params_file:=$HOME/my_params.yaml

Publishes /<camera_name>/flow_image (sensor_msgs/Image) -- view it in
rqt_image_view. The driver (built on openeb_vendor) owns the camera and your
params_file (ERC cap, biases) governs the event rate. The flow node links the
SDK Pro 5.x build, a different SDK version than the driver, so it runs as its
OWN process and consumes events over the topic (DDS). The launch puts the SDK's
libs first on the flow node's LD_LIBRARY_PATH automatically (path captured at
build time) -- no `source setup_env.sh` needed, and the driver's process is left
on openeb_vendor untouched.
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
    debug_timing = LaunchConfiguration('debug_timing').perform(context).lower() == 'true'
    params_file = LaunchConfiguration('params_file').perform(context)

    # The flow node links the SDK Pro 5.x build tree, whose libraries use RUNPATH
    # -- so their transitive deps follow LD_LIBRARY_PATH. We put that lib dir
    # (captured into the package share at build time) FIRST on the flow node's
    # LD_LIBRARY_PATH, keeping its whole process on 5.x instead of leaking to
    # openeb_vendor's 5.0.0. The flow node does not open the camera, so it needs
    # no HAL plugins; the driver keeps its own (openeb_vendor) environment.
    sdk_libdir = ''
    libdir_file = os.path.join(
        get_package_share_directory('evk4_sdk_advanced'), 'sdk_libdir')
    if os.path.exists(libdir_file):
        with open(libdir_file) as handle:
            sdk_libdir = handle.read().strip()
    flow_env = {}
    if sdk_libdir:
        flow_env['LD_LIBRARY_PATH'] = (
            sdk_libdir + os.pathsep + os.environ.get('LD_LIBRARY_PATH', ''))

    # Driver in its own process (reuses the canonical bringup; renderer off).
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

    flow = Node(
        package='evk4_sdk_advanced',
        executable='optical_flow',
        name='optical_flow',
        namespace=camera_name,
        parameters=[{'fps': fps, 'debug_timing': debug_timing}],
        # Absolute remaps so topic resolution does not depend on namespace.
        remappings=[
            ('events', f'/{camera_name}/events'),
            ('flow_image', f'/{camera_name}/flow_image'),
        ],
        additional_env=flow_env,
        output='screen',
    )

    return [driver, flow]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name', default_value='event_camera',
            description='Node name and topic namespace for the camera.'),
        DeclareLaunchArgument(
            'serial', default_value='',
            description='Camera serial number (empty: first camera found).'),
        DeclareLaunchArgument(
            'frame_id', default_value='event_camera_optical_frame',
            description='TF frame stamped on the flow image.'),
        DeclareLaunchArgument(
            'fps', default_value='30.0',
            description='Flow image frame rate in Hz.'),
        DeclareLaunchArgument(
            'debug_timing', default_value='false',
            description='Log per-second per-stage processing times for profiling.'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Driver params YAML (default: evk4_bringup '
                        'config/evk4_params.yaml). Use your ~/my_params.yaml.'),
        OpaqueFunction(function=_launch_setup),
    ])

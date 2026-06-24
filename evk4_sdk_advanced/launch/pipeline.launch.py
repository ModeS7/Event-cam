"""Run any evk4_sdk_advanced pipeline on the camera.

    ros2 launch evk4_sdk_advanced pipeline.launch.py pipeline:=dense_flow \
        params_file:=$HOME/my_params.yaml

`pipeline` is one of: optical_flow, tracking, dense_flow, spatter, counting,
frequency, led_tracking, psm, jet_monitoring, undistortion, edgelet. The ML-tier pipelines
(x86 + GPU build only) are gesture, detection, flow_inference -- pass their
`model_path` (a `.ptjit`) and `gpu_id` via `node_params_file:=...`. Each publishes
/<camera_name>/<pipeline>_image. The
driver (openeb_vendor) runs in its own process; the SDK node runs separately
with the SDK libs on its LD_LIBRARY_PATH (captured at build time -- no
`source setup_env.sh` needed). Pipeline-specific params (min_size, radius,
cell_size, min_freq, led_tracking base_period_us, ...) keep their node defaults;
pass a YAML with `node_params_file:=...` to override them, e.g.:

    ros2 launch evk4_sdk_advanced pipeline.launch.py pipeline:=led_tracking \
        params_file:=$HOME/my_params.yaml node_params_file:=/tmp/led.yaml

where /tmp/led.yaml is `/**:` -> `ros__parameters:` -> base_period_us: 5000 ...
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
    pipeline = LaunchConfiguration('pipeline').perform(context)
    camera_name = LaunchConfiguration('camera_name').perform(context)
    serial = LaunchConfiguration('serial').perform(context)
    file_path = LaunchConfiguration('file').perform(context)
    frame_id = LaunchConfiguration('frame_id').perform(context)
    fps = float(LaunchConfiguration('fps').perform(context))
    debug_timing = LaunchConfiguration('debug_timing').perform(context).lower() == 'true'
    params_file = LaunchConfiguration('params_file').perform(context)
    node_params_file = LaunchConfiguration('node_params_file').perform(context)
    calibration_url = LaunchConfiguration('calibration_url').perform(context)
    topic = f'{pipeline}_image'

    # Pipeline-specific node parameters (e.g. led_tracking base_period_us,
    # frequency min_freq) -- a YAML applied to the node. Use `/**:` to match.
    # calibration_url is only read by the undistortion pipeline; others ignore it.
    node_parameters = [{
        'fps': fps, 'debug_timing': debug_timing, 'calibration_url': calibration_url}]
    if node_params_file:
        node_parameters.append(node_params_file)

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
            'file': file_path,
            'frame_id': frame_id,
            'viz': 'false',
            'params_file': params_file,
            # Our calibration_url is for the undistortion NODE; force it empty for
            # the driver launch so its CameraInfoPublisher (which needs viz) is not
            # started. Included launches otherwise inherit this configuration.
            'calibration_url': '',
        }.items(),
    )

    node = Node(
        package='evk4_sdk_advanced',
        executable=pipeline,
        name=pipeline,
        namespace=camera_name,
        parameters=node_parameters,
        remappings=[
            ('events', f'/{camera_name}/events'),
            (topic, f'/{camera_name}/{topic}'),
        ],
        additional_env=node_env,
        output='screen',
    )

    return [driver, node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'pipeline',
            description='optical_flow | tracking | dense_flow | spatter | counting | '
                        'frequency | led_tracking | psm | jet_monitoring | undistortion | '
                        'edgelet (cv3d tier, needs USE_SOPHUS=ON) | '
                        'gesture | detection | flow_inference (ML tier, x86+GPU)'),
        DeclareLaunchArgument('camera_name', default_value='event_camera'),
        DeclareLaunchArgument('serial', default_value=''),
        DeclareLaunchArgument(
            'file', default_value='',
            description='Replay a recorded RAW (EVT3) file instead of the live '
                        'camera -- e.g. a driving clip for the detection pipeline.'),
        DeclareLaunchArgument('frame_id', default_value='event_camera_optical_frame'),
        DeclareLaunchArgument('fps', default_value='30.0'),
        DeclareLaunchArgument('debug_timing', default_value='false'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Driver params YAML (use your ~/my_params.yaml).'),
        DeclareLaunchArgument(
            'node_params_file', default_value='',
            description='Pipeline node params YAML, e.g. led_tracking '
                        'base_period_us / inactivity_period_us (match a `/**:` block).'),
        DeclareLaunchArgument(
            'calibration_url', default_value='',
            description='camera_info YAML (from evk4_calibration) -- required by the '
                        'undistortion pipeline, ignored by the others.'),
        OpaqueFunction(function=_launch_setup),
    ])

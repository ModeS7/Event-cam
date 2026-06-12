"""Bring up the Prophesee EVK4: our evk4_driver plus optional renderer.

Topic contract (with default arguments):

    /event_camera/events       event_camera_msgs/msg/EventPacket   (always)
    /event_camera/image_raw    sensor_msgs/msg/Image               (viz:=true)
    /event_camera/camera_info  sensor_msgs/msg/CameraInfo          (calibration_url set)
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
    """Fail loudly at launch time if a required package is missing.

    Our own packages (evk4_*) are built from this repo; everything else is an
    apt/source dependency, with the hint derived from $ROS_DISTRO (so it is
    correct on any distro, not hardcoded to one).
    """
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
    serial = LaunchConfiguration('serial').perform(context)
    frame_id = LaunchConfiguration('frame_id').perform(context)
    sync_mode = LaunchConfiguration('sync_mode').perform(context)
    trigger_in_mode = LaunchConfiguration('trigger_in_mode').perform(context)
    settings = LaunchConfiguration('settings').perform(context)
    calibration_url = LaunchConfiguration('calibration_url').perform(context)
    rectify = LaunchConfiguration('rectify').perform(context).lower()
    viz = LaunchConfiguration('viz').perform(context).lower()
    if viz not in ('true', 'false'):
        raise RuntimeError(f"viz must be 'true' or 'false', got '{viz}'")
    fps = float(LaunchConfiguration('fps').perform(context))
    display_type = LaunchConfiguration('display_type').perform(context)

    _require('evk4_driver')
    # params_file holds the long tail of driver params; default to ours, but
    # let users swap in their own without editing the package.
    params_file = LaunchConfiguration('params_file').perform(context) or os.path.join(
        get_package_share_directory('evk4_bringup'), 'config', 'evk4_params.yaml')

    # Driver and renderer are composed into ONE container process with
    # intra-process communication: the high-rate event stream passes
    # between them as pointers instead of being serialized and copied
    # through the middleware. Subscribers in other processes (your nodes,
    # rosbag2, rqt) still receive normal DDS copies.
    components = [
        ComposableNode(
            package='evk4_driver',
            plugin='evk4_driver::EVK4Driver',
            name=camera_name,
            # Our OpenEB-based driver honors frame_id (unlike metavision_driver
            # 3.0.0, which ignored it and stamped the serial tail).
            # sync_mode (standalone/primary/secondary) hardware-syncs multiple
            # cameras over the sync cable; trigger_in_mode records an external
            # trigger pin for syncing with other sensors (IMU/RGB/...).
            parameters=[
                params_file,
                {
                    'serial': serial,
                    'frame_id': frame_id,
                    'sync_mode': sync_mode,
                    'trigger_in_mode': trigger_in_mode,
                    'settings': settings,
                },
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]
    if viz == 'true':
        _require('event_camera_renderer')
        # Precedence: defaults < the params YAML < explicitly passed launch
        # args — one file can carry the whole setup (docs/tuning.md) while
        # fps:= / display_type:= stay quick one-off overrides ("explicit" =
        # differs from the default).
        renderer_params = [
            {'fps': 25.0, 'display_type': 'time_slice'}, params_file]
        overrides = {}
        if fps != 25.0:
            overrides['fps'] = fps
        if display_type != 'time_slice':
            overrides['display_type'] = display_type
        if overrides:
            renderer_params.append(overrides)
        components.append(
            ComposableNode(
                package='event_camera_renderer',
                plugin='event_camera_renderer::Renderer',
                name='renderer',
                # namespace under camera_name so two cameras don't both create
                # a node literally named /renderer (multi-camera safety).
                namespace=camera_name,
                parameters=renderer_params,
                remappings=[
                    ('~/events', f'/{camera_name}/events'),
                    ('~/image_raw', f'/{camera_name}/image_raw'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ))

    # camera_info enables image_proc rectification. It is derived from
    # image_raw, so it only makes sense when the renderer runs. Composed into
    # the same container so images arrive intra-process (pointer, no copy).
    if calibration_url:
        if viz != 'true':
            raise RuntimeError(
                'calibration_url needs the rendered image; run with viz:=true')
        components.append(
            ComposableNode(
                package='evk4_bringup',
                plugin='evk4_bringup::CameraInfoPublisher',
                name='camera_info_publisher',
                namespace=camera_name,
                parameters=[{'calibration_url': calibration_url}],
                remappings=[('image_raw', f'/{camera_name}/image_raw'),
                            ('camera_info', f'/{camera_name}/camera_info')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ))

    # Optional in-container rectification: the image reaches image_proc as a
    # pointer (no serialization), only the rectified output leaves the
    # container for subscribers.
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
                # Nearest-neighbor remap: ~60 ms/frame cheaper than the
                # bilinear default on a Pi (validated 2026-06-12), and
                # event images are sparse hard-edged pixels — bilinear
                # only smears them.
                parameters=[{'interpolation': 0}],
                remappings=[('image', f'/{camera_name}/image_raw'),
                            ('camera_info', f'/{camera_name}/camera_info'),
                            ('image_rect', f'/{camera_name}/image_rect')],
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
    """Declare arguments and hand off to _launch_setup.

    Node construction lives in an OpaqueFunction because it needs the
    *resolved* argument values (to validate `viz`/`fps` and branch on them) —
    those are only available with a launch context, not at description-build
    time.
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name', default_value='event_camera',
            description='Node name and topic namespace for the camera.'),
        DeclareLaunchArgument(
            'serial', default_value='',
            description='Camera serial number (empty: first camera found).'),
        DeclareLaunchArgument(
            'frame_id', default_value='event_camera_optical_frame',
            description='TF frame for camera messages (driver 3.0.0 ignores '
                        'this and uses the serial tail; see docs/calibration.md).'),
        DeclareLaunchArgument(
            'sync_mode', default_value='standalone',
            description='Hardware sync: standalone | primary | secondary '
                        '(multi-camera; see docs/multi_camera.md).'),
        DeclareLaunchArgument(
            'trigger_in_mode', default_value='disabled',
            description='External trigger input: disabled | external | '
                        'loopback (timestamp sync with other sensors).'),
        DeclareLaunchArgument(
            'settings', default_value='',
            description='Path to a camera settings JSON (e.g. pixel masks); '
                        'also the target for the save_settings service.'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Override the driver params YAML (default: the package '
                        "config/evk4_params.yaml). Escape hatch for any driver param."),
        DeclareLaunchArgument(
            'viz', default_value='true',
            description='Also run event_camera_renderer, publishing '
                        '<camera_name>/image_raw.'),
        DeclareLaunchArgument(
            'fps', default_value='25.0',
            description='Renderer frame rate in Hz (only with viz:=true).'),
        DeclareLaunchArgument(
            'display_type', default_value='time_slice',
            description='Renderer mode: time_slice or sharp.'),
        DeclareLaunchArgument(
            'calibration_url', default_value='',
            description='Path to a camera_info YAML; when set, publish '
                        '<camera_name>/camera_info for image_proc rectification.'),
        DeclareLaunchArgument(
            'rectify', default_value='false',
            description='Also publish <camera_name>/image_rect (undistorted), '
                        'composed in-container; needs calibration_url.'),
        OpaqueFunction(function=_launch_setup),
    ])

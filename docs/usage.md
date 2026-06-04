# Usage

## Launching the camera

```bash
ros2 launch evk4_bringup evk4.launch.py
```

| Argument | Default | Description |
|---|---|---|
| `camera_name` | `event_camera` | Node name and topic namespace |
| `serial` | `''` (first camera) | Select a camera by serial number |
| `bias_file` | `''` (sensor defaults) | Path to a `.bias` file to load |
| `viz` | `true` | Also run the renderer (image topic) |

Examples:

```bash
ros2 launch evk4_bringup evk4.launch.py viz:=false           # raw events only
ros2 launch evk4_bringup evk4.launch.py serial:=00050591     # pick a camera
ros2 launch evk4_bringup evk4.launch.py bias_file:=~/my.bias # custom biases
```

Driver tuning parameters (event rate control, noise filtering, batching)
live in `evk4_bringup/config/evk4_params.yaml` — see the comments there and
the [upstream parameter reference](https://github.com/ros-event-camera/metavision_driver).

## Topics

| Topic | Type | When |
|---|---|---|
| `/event_camera/events` | `event_camera_msgs/msg/EventPacket` (EVT3) | always |
| `/event_camera/image_raw` | `sensor_msgs/msg/Image` (default 25 fps) | `viz:=true` |

## Services (provided by the driver)

| Service | Type | Effect |
|---|---|---|
| `/event_camera/save_biases` | `std_srvs/srv/Trigger` | Save current biases (uses the `bias_file` path) |
| `/event_camera/save_settings` | `std_srvs/srv/Trigger` | Save current camera settings |
| `/event_camera/dump_statistics` | `std_srvs/srv/Trigger` | Log bandwidth/rate statistics |

```bash
ros2 service call /event_camera/dump_statistics std_srvs/srv/Trigger
```

## Consuming events

> **QoS:** the driver publishes **best-effort**. A subscriber with default
> (reliable) QoS will match nothing and receive nothing. In Python use
> `rclpy.qos.qos_profile_sensor_data`; in C++ use
> `rclcpp::SensorDataQoS()`.

### Python (recommended starting point)

Run the included example, which decodes packets with
[event_camera_py](https://github.com/ros-event-camera/event_camera_py) and
logs event rates:

```bash
ros2 run evk4_examples event_rate
# non-default camera name:
ros2 run evk4_examples event_rate --ros-args -r event_camera/events:=/my_camera/events
```

The decoding core, for your own nodes
(see [`event_rate_node.py`](../evk4_examples/evk4_examples/event_rate_node.py)):

```python
from event_camera_py import Decoder

decoder = Decoder()
decoder.decode(msg)                    # msg: event_camera_msgs/EventPacket
cd_events = decoder.get_cd_events()    # numpy array, fields: x, y, p, t [us]
```

### C++

Use [event_camera_codecs](https://github.com/ros-event-camera/event_camera_codecs)
to decode `EventPacket` messages; its README has a complete subscriber
example.

## Visualization

With `viz:=true` (default):

```bash
ros2 run rqt_image_view rqt_image_view /event_camera/image_raw
```

The renderer runs with its defaults (25 fps, `time_slice` display). For
other settings, run it standalone:
`ros2 launch event_camera_renderer renderer.launch.py camera:=event_camera fps:=60.0`

## Bias tuning

Biases are the sensor's analog settings (contrast thresholds, bandwidth,
…). The factory defaults are a good start; tune only with a reason. See the
[driver README](https://github.com/ros-event-camera/metavision_driver) and
Prophesee's bias documentation for the workflow, then load your file with
`bias_file:=...` (store files under `evk4_bringup/config/biases/`).

## Recording and playback

Record raw event packets (compact — this is the EVT3 stream, not images):

```bash
ros2 bag record /event_camera/events
```

Play back and analyze offline:

```bash
ros2 bag play <bag>                    # republished on /event_camera/events
ros2 run evk4_examples event_rate      # or your own subscriber
```

`event_camera_py` can also read bags directly in a script (no ROS graph
needed) — see its README. For conversion to/from Prophesee `.raw` files and
other utilities, see
[event_camera_tools](https://github.com/ros-event-camera/event_camera_tools).

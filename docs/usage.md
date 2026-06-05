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

**Architecture note:** the launch composes driver and renderer into a single
container process with intra-process communication, so the high-rate event
stream is passed by pointer between them — never serialized or copied.
Subscribers in *other* processes (your nodes, rosbag2, rqt) receive normal
DDS copies; if you write a high-throughput C++ consumer, implement it as a
composable component and load it into the same container to get the same
zero-copy path.

## Topics

| Topic | Type | When |
|---|---|---|
| `/event_camera/events` | `event_camera_msgs/msg/EventPacket` (EVT3) | always |
| `/event_camera/image_raw` | `sensor_msgs/msg/Image` (default 25 fps) | `viz:=true` |

Message headers carry `frame_id` = last 4 digits of the camera serial
number (e.g. `1701`) — driver 3.0.0 does not support overriding it.

## Services (provided by the driver, v3.0.0)

| Service | Type | Effect |
|---|---|---|
| `/event_camera/save_biases` | `std_srvs/srv/Trigger` | Save current biases (uses the `bias_file` path) |
| `/event_camera/save_settings` | `std_srvs/srv/Trigger` | Save current camera settings |

```bash
ros2 service call /event_camera/save_biases std_srvs/srv/Trigger
```

Services only exist while the camera launch is running. For bandwidth and
rate statistics no service is needed: the driver prints them to the launch
terminal every second (`statistics_print_interval` parameter), e.g.

```
[event_camera]: bw in: 9.75 MB/s, msgs/s in: 247, out: 247, maxq: 1
```

(`out` counts messages delivered to subscribers in other processes — it is
0 when nothing external is subscribed.)

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

`evk4_examples_cpp` contains the same example in C++
([`src/event_rate.cpp`](../evk4_examples_cpp/src/event_rate.cpp)), decoding
with [event_camera_codecs](https://github.com/ros-event-camera/event_camera_codecs):

```bash
ros2 run evk4_examples_cpp event_rate
```

It is built as a **composable component**, so for the zero-copy
intra-process path you can load it into the running camera container
instead of starting a separate process:

```bash
ros2 component load /event_camera_container evk4_examples_cpp \
    evk4_examples_cpp::EventRate -e use_intra_process_comms:=true
```

(With the container loaded this way, the driver's `out` statistics counter
stays at 0 for this subscriber — events are handed over as pointers, not
published through DDS. Use this pattern for your own high-rate consumers.)

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
ros2 bag record --topics /event_camera/events
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

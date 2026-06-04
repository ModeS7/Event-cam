# Bias files

Place EVK4 `.bias` files here and point the `bias_file` launch argument at
them. None are committed by default — the sensor's factory biases are a good
starting point. Save the current camera biases at runtime with:

```bash
ros2 service call /event_camera/save_biases std_srvs/srv/Trigger
```

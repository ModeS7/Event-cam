# Bias files

Biases are the EVK4's analog sensor settings (contrast thresholds
`bias_diff_on`/`bias_diff_off`, bandwidth `bias_fo`, high-pass `bias_hpf`,
refractory period `bias_refr`). Factory defaults are a good starting point;
this directory is the conventional home for tuned `.bias` files (none are
committed by default).

To create one:

```bash
# 1. Launch with bias_file pointing where the file should go.
#    If it doesn't exist yet, the driver warns and uses factory defaults.
ros2 launch evk4_bringup evk4.launch.py bias_file:=/path/to/biases/my.bias

# 2. Tune biases live while watching the image / event rate:
ros2 param set /event_camera bias_diff_on 30

# 3. Write the current values to the bias_file path:
ros2 service call /event_camera/save_biases std_srvs/srv/Trigger
```

If the launch did not set `bias_file`, the service returns `success=False`
with `message: 'no bias file specified at startup'`. Subsequent launches
with the same `bias_file` load the saved values.

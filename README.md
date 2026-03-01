````markdown
# zenorak_controller_motor

A clear, well-documented ros2_control hardware plugin for a differential-drive robot that talks to a microcontroller (for example, an Arduino) over serial. This package exposes two wheel joints to ros2_control and converts between controller velocity commands and microcontroller motor counts. It also reads encoder counts from the microcontroller and publishes them as joint states via ros2_control state interfaces.

This README explains how the package is structured, how the code works, and how to integrate it into a URDF/xacro-based robot description so controllers can use it.

---

## What this package provides (high level)

- A SystemInterface implementation (the hardware plugin) that exposes the following interfaces to ros2_control:
  - left_wheel_joint: position (state), velocity (state), velocity (command)
  - right_wheel_joint: position (state), velocity (state), velocity (command)
- A tiny serial helper (`ArduinoComms`) that sends simple ASCII commands to the microcontroller and reads ASCII replies.
- Small helper classes (`Wheel`, optional `Actuator`) used to store runtime state (encoder counts, position, velocity, command).

The plugin is intended to be used with standard ROS 2 controllers (for example, `diff_drive_controller`), launched from a bringup/launch file and referenced from a URDF/xacro 'ros2_control' hardware block.

---

## Key files and what they do (developer-friendly)

- `hardware/diffbot_system.cpp` - The core hardware plugin implementation. It implements the lifecycle callbacks (`on_init`, `on_configure`, `on_activate`, `read`, `write`) used by ros2_control. This file contains the logic that:

  - reads the configuration from the `ros2_control` hardware block (serial device, baud rate, joint names, encoder counts)
  - converts encoder counts read from the microcontroller into joint positions and velocities
  - converts commanded wheel velocities into "counts per loop" and sends motor commands to the microcontroller

- `hardware/include/zenorak_controller_motor/arduino_comms.hpp` - Serial communication helper. It exposes `connect()`, `disconnect()`, `read_encoder_values()`, and `set_motor_values()`.

  - Protocol expected by microcontroller (ASCII commands):
    - `e\r` -> returns: `<left_enc> <right_enc>\n`
    - `m <left_counts> <right_counts>\r` -> set motor counts (no reply expected)

- `hardware/include/zenorak_controller_motor/wheel.hpp` - Small struct-like class used to store `name`, `enc`, `cmd`, `pos`, `vel`, and conversion factor `rads_per_count`.

- `hardware/include/zenorak_controller_motor/Actuator.hpp` - Optional helper mapping potentiometer values to angles. If your robot has no analog actuators, this file is unused but left for reference.

- `description/ros2_control/diffbot.ros2_control.xacro` - Example `ros2_control` hardware block and joint entries to include in your robot's URDF/xacro.

- `bringup/launch/diffbot.launch.py` and `bringup/config/diffbot_controllers.yaml` - Example launch and controllers configuration to run the hardware with controllers.

---

## How it works (step-by-step, plain English)

1. At startup, ROS 2 loads the hardware plugin and calls `on_init()` with parameters defined in the robot's `ros2_control` xacro/URDF hardware block.
2. The plugin stores configuration such as joint names, serial device path (e.g. `/dev/ttyACM0`), baud rate, encoder counts per revolution, and loop rate.
3. When the lifecycle transitions to `configure`, the plugin opens the serial port to the microcontroller.
4. In the running state, `read()` is called periodically by the controller manager:
   - It sends `e\r` to the microcontroller and expects two integers (left and right encoder counts).
   - It converts counts -> radians via `rads_per_count` and updates the ROS state interfaces (position & velocity).
5. Also while running, `write()` is called periodically:
   - It reads velocity commands (rad/s) from the command interfaces.
   - It converts these to counts-per-loop using `counts_per_loop = cmd / rads_per_count / loop_rate`.
   - It sends `m <left_counts> <right_counts>\r` to the microcontroller.

Output visible to the rest of ROS:

- The plugin provides state interfaces (position and velocity) which controllers and `joint_state_broadcaster` can read and publish. Controllers write velocity commands which eventually get sent to the microcontroller as motor counts.

---

## How to build

From your ROS 2 workspace root (where `src/` is located):

```bash
colcon build --packages-select zenorak_controller_motor
```
````

Source the install overlay after build:

```bash
source install/setup.bash
```

---

## Example: how to include this hardware in your URDF/xacro

Below is a minimal snippet to add to your robot xacro that tells ros2_control to use this plugin.

```xml
<ros2_control name="diffbot_hw" type="system">
	<hardware>
		<plugin>zenorak_controller_motor/Zenorak_Hardware_Motor</plugin>
		<param name="left_wheel_name">left_wheel_joint</param>
		<param name="right_wheel_name">right_wheel_joint</param>
		<param name="loop_rate">10</param>
		<param name="device">/dev/ttyACM0</param>
		<param name="baud_rate">115200</param>
		<param name="timeout_ms">1000</param>
		<param name="enc_counts_per_rev">32</param>
	</hardware>

	<!-- define the two wheel joints and their interfaces -->
	<joint name="left_wheel_joint">
		<command_interface name="velocity"/>
		<state_interface name="position"/>
		<state_interface name="velocity"/>
	</joint>
	<joint name="right_wheel_joint">
		<command_interface name="velocity"/>
		<state_interface name="position"/>
		<state_interface name="velocity"/>
	</joint>
</ros2_control>
```

Notes:

- `loop_rate` must match the control loop frequency (how often `read()`/`write()` are called).
- `enc_counts_per_rev` must match your encoder hardware.

---

## Example controller configuration

In `bringup/config/diffbot_controllers.yaml` you'll typically have a `diff_drive_controller` and broadcaster entries. Example:

```yaml
controller_manager:
	ros__parameters:
		update_rate: 50

joint_state_broadcaster:
	ros__parameters:
		use_sim_time: false

diff_drive_controller:
	ros__parameters:
		left_wheel: left_wheel_joint
		right_wheel: right_wheel_joint
		wheel_separation: 0.5
		wheel_radius: 0.1
		cmd_vel_timeout: 0.5
```

Adjust `wheel_separation` and `wheel_radius` to your robot.

---

## How to run (basic)

1. Build and source your workspace.
2. Launch your robot description (URDF/xacro) that contains the `ros2_control` block referencing this plugin. Example (adjust paths):

```bash
ros2 launch zenorak_controller_motor bringup/launch/diffbot.launch.py
```

3. Start controllers (example using ros2 control CLI):

```bash
# list available controllers
ros2 control list_controllers
# load and start joint_state_broadcaster and diff_drive_controller as appropriate
ros2 control load_start_controller joint_state_broadcaster
ros2 control load_start_controller diff_drive_controller
```

4. Publish velocity commands (example):

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}' -r 10
```

You should see the microcontroller receive `m <left> <right>` commands and the `joint_states` topic reflect wheel positions.

---

## Microcontroller (Arduino) expectations

This plugin uses a simple ASCII protocol. The microcontroller firmware should implement at least:

- On receiving `e\r` respond with: `<left_enc> <right_enc>\n` (two signed integers and a newline).
- On receiving `m <left> <right>\r` set motor speed/position accordingly (no response required).

A robust microcontroller implementation should also handle invalid input gracefully and send timely responses.

---

## Troubleshooting

- If the plugin fails to configure, check that the serial device exists and that you have permission to open it.
- If you see timeouts, increase `timeout_ms` in the ros2_control hardware params or debug the microcontroller firmware to ensure it replies.
- Use `ros2 topic echo /joint_states` to inspect published wheel positions.

---

## Developer notes (production-friendly comments)

- The hardware plugin is intentionally small and synchronous. For production systems, consider:

  - Offloading serial reads/writes to a dedicated thread and using lock-free buffers for real-time safety.
  - Adding better error handling and health reporting for serial comms.
  - Instrumentation for latency and packet loss.

- This package keeps logic simple so it is easy to adapt the serial protocol if your microcontroller uses different messages.

---

## License

Licensed under Apache-2.0 (same as upstream ros2_control demos).

**_ End of README _**

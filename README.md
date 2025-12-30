zenorak_controller_actuator - Full README

## Overview

`zenorak_controller_actuator` provides a ros2_control SystemInterface (hardware plugin) that bridges ROS 2 controllers to simple actuator hardware driven by a microcontroller (for example an Arduino). The package focuses on a 3-DOF linear actuator manipulator and exposes position state and position command interfaces for each actuator.

Key features

- Serial communication to a microcontroller (read ADC/potentiometer values and send actuator target counts).
- `joint_state_broadcaster` and `joint_trajectory_controller` friendly interfaces (position state + position command).
- Example URDF/ros2_control macro to include this hardware plugin into a robot description.

## Build

Build this package inside a ROS 2 workspace (source ROS 2 and use colcon):

```bash
# from the workspace root (where `src/` lives)
colcon build --packages-select zenorak_controller_actuator
source install/setup.bash
```

## Run / Launch

An example launch is provided at `bringup/launch/diffbot.launch.py`. It will:

- Expand the robot URDF/Xacro (`urdf/diffbot.urdf.xacro`) and pass it to `ros2_control_node`.
- Start the `ros2_control_node` (controller manager) with the controllers configured in `bringup/config/diffbot_controllers.yaml`.
- Spawn the `joint_state_broadcaster` and the `arm_trajectory_controller`.

To launch the example:

```bash
ros2 launch zenorak_controller_actuator diffbot.launch.py
```

If you need to remap where joint states appear, you can either update the launch file to remap `/joint_states` (recommended) or publish your own `sensor_msgs/msg/JointState` to the appropriate topic. See the docs in `docs/annotated/` for guidance on remapping.

## How it integrates into a URDF (ros2_control)

1. In your robot URDF/Xacro, include the `diffbot.ros2_control.xacro` macro (found under `description/ros2_control/`) and instantiate it with the desired joint names and parameters. Example snippet:

```xml
  <xacro:diffbot_ros2_control name="zenorak_hw" prefix="" link1_name="link_1_joint" link2_name="link_2_joint" link3_name="link_3_joint"/>
```

2. The macro sets the `<ros2_control>` `<hardware>` plugin name to `zenorak_controller_actuator/Zenorak_Hardware_Actuator`. When `ros2_control_node` loads the URDF, it will load our hardware plugin automatically.

3. Configure controllers in `bringup/config/diffbot_controllers.yaml`. Ensure joint names in the controller YAML match those provided to the URDF macro.

4. Launch the controller manager using the provided launch file or your own launch that supplies the `robot_description` parameter.

## Topics, messages and commands

- `/joint_states` (published by `joint_state_broadcaster`): `sensor_msgs/msg/JointState` containing the positions (and optionally velocities/efforts) of `link_1_joint`, `link_2_joint`, `link_3_joint`.
- Controller manager services and topics: standard ros2_control controller manager API (use `ros2 control` CLI to manage controllers).

## How other packages can use this hardware

To use this hardware plugin from another package (for example your own robot's URDF):

1. Add `zenorak_controller_actuator` as a dependency in your package.xml and CMakeLists if you reference its files directly.
2. Include the `diffbot.ros2_control.xacro` macro in your URDF or copy its pattern to your robot's `<ros2_control>` tag and set the plugin to `zenorak_controller_actuator/Zenorak_Hardware_Actuator`.
3. Provide the hardware parameters (`device`, `baud_rate`, `timeout_ms`, `link1_name`, etc.) as `<param>` entries inside the `<hardware>` tag.
4. Ensure your controllers' joint names match what the hardware will export.

Example: minimal `<ros2_control>` snippet for your URDF:

```xml
<ros2_control name="my_robot_hw" type="system">
  <hardware>
    <plugin>zenorak_controller_actuator/Zenorak_Hardware_Actuator</plugin>
    <param name="device">/dev/ttyACM0</param>
    <param name="baud_rate">115200</param>
    <param name="timeout_ms">1000</param>
    <param name="link1_name">link_1_joint</param>
    <param name="link2_name">link_2_joint</param>
    <param name="link3_name">link_3_joint</param>
  </hardware>
  <!-- per-joint entries follow -->
</ros2_control>
```

## Troubleshooting and tips

- Serial port permissions: ensure your user can open `/dev/ttyACM0` (add to `dialout` group or use `udev` rules).
- If readings look wrong: check ADC mapping parameters (`linkX_enc_val`) and the ranges used in `Actuator::potToAngle`.
- If controllers fail to start: confirm joint names match across URDF, controller YAML, and hardware plugin logs.

## Annotated sources and developer docs

Line-by-line annotated versions of key sources are available under `docs/annotated/`:

- `docs/annotated/diffbot_system.cpp.annotated.md`
- `docs/annotated/arduino_comms.hpp.annotated.md`
- `docs/annotated/Actuator.hpp.annotated.md`
- `docs/annotated/diffbot.launch.py.annotated.md`
- `docs/annotated/diffbot_controllers.yaml.annotated.md`
- `docs/annotated/diffbot.ros2_control.xacro.annotated.md`

These files explain what each line does, where it is called from, expected inputs and outputs, and production notes. Use them to onboard teammates or to adapt the code to new hardware.

## Contributing

- If you extend the serial protocol on the microcontroller, update `arduino_comms.hpp` to match the new command/response strings and update the annotated docs.
- For high-frequency or production control, consider moving serial I/O to a dedicated thread and adding robust parsing/validation of messages.

## License

This package uses the Apache 2.0 license (see headers in source files).

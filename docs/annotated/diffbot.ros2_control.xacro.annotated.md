# Annotated: diffbot.ros2_control.xacro

This file contains the `<ros2_control>` system definition used by the URDF/Xacro to tell `ros2_control_node` which plugin to load and which joints to export. Below are line-level explanations and usage notes.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="diffbot_ros2_control" params="name prefix link1_name='link_1_joint' link2_name='link_2_joint' link3_name='link_3_joint'">

    <ros2_control name="${name}" type="system">
      <hardware>
        <plugin>zenorak_controller_actuator/Zenorak_Hardware_Actuator</plugin>
```

- What it is: Declares the hardware plugin that the controller manager should load. The string `zenorak_controller_actuator/Zenorak_Hardware_Actuator` corresponds to the pluginlib export in `diffbot_system.cpp` (PLUGINLIB_EXPORT_CLASS).
- Where this string is used: `ros2_control_node` reads this tag and uses pluginlib to construct the hardware system.

```xml
        <!-- Wheel/motor parameters removed - only actuator params kept -->
        <param name="device">/dev/ttyACM0</param>
        <param name="baud_rate">115200</param>
        <param name="timeout_ms">1000</param>
        <!-- enc_counts_per_rev removed -->
        <param name="pid_p">20</param>
        <param name="pid_d">12</param>
        <param name="pid_i">0</param>
        <param name="pid_o">50</param>
```

- These parameters are read by `Zenorak_Hardware_Actuator::on_init()` and used to configure the `ArduinoComms` and PID settings.

```xml
        <param name="link1_name">${link1_name}</param>
        <param name="link2_name">${link2_name}</param>
        <param name="link3_name">${link3_name}</param>
        <param name="link1_enc_val">1023</param>
        <param name="link2_enc_val">1023</param>
        <param name="link3_enc_val">1023</param>
```

- `linkX_enc_val` are the ADC full-scale values (e.g., 10-bit ADC = 1023) used in `Actuator.setup` or conversion logic.

```xml
      </hardware>

      <!-- Manipulator joints: these should match the joint names in your robot URDF/xacro -->
      <joint name="${prefix}${link1_name}">
        <command_interface name="position"/>
        <state_interface name="position"/>
      </joint>

      <joint name="${prefix}${link2_name}">
        <command_interface name="position"/>
        <state_interface name="position"/>
      </joint>

      <joint name="${prefix}${link3_name}">
        <command_interface name="position"/>
        <state_interface name="position"/>
      </joint>
```

- These `<joint>` entries are how ros2_control knows which joints to expose and which interfaces to expect. Names must match the actual joint elements used elsewhere in your URDF (the physical joint frames).

Cross references:

- The plugin name corresponds to the `PLUGINLIB_EXPORT_CLASS` macro in `hardware/diffbot_system.cpp`.
- `link1_name`, `link2_name`, `link3_name` must match joint names used in `urdf/diffbot_description.urdf.xacro`.

Production guidance:

- When you instantiate this macro in your URDF (for example, in `diffbot.urdf.xacro`), pass a unique prefix if you have multiple robots in the same ROS graph to avoid joint name collisions.
- Keep param names consistent if you write scripts to generate robot descriptions programmatically.

# Annotated: diffbot.launch.py

This file launches the `ros2_control_node` and spawners for controllers used by the package. Below we explain the important parts and how the broadcaster/controller spawners work.

```python
# Copyright 2020 ros2_control Development Team
# (License header omitted for brevity in this annotated view)

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
```

- Imports: these bring in ROS 2 launch primitives used to build the launch description that starts multiple nodes together.

```python
def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("zenorak_controller_actuator"), "urdf", "diffbot.urdf.xacro"]),
        ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
```

- What it is: expands the `diffbot.urdf.xacro` into a plain URDF string at launch time and passes it as the `robot_description` parameter. This is a common ROS pattern to avoid creating static URDF files.

```python
    robot_controllers = PathJoinSubstitution([
        FindPackageShare("zenorak_controller_actuator"),
        "config",
        "diffbot_controllers.yaml",
    ])
```

- Path to controller config YAML used by controller_manager.

```python
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
```

- Starts `ros2_control_node` (the controller manager) with the URDF and controller YAML. This node will load the hardware plugin specified in the URDF and then create joint state and trajectory controllers according to the YAML.

```python
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
```

- `spawner` is a helper executable that requests the controller manager to load and start a controller by name. This line spawns `joint_state_broadcaster` which publishes `/joint_states` (or the remapped topic if you adjusted it) using the state interfaces exported by the hardware.

```python
    arm_trajectory_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_trajectory_controller", "--controller-manager", "/controller_manager"],
    )
```

- Spawns the group trajectory controller that accepts position commands for the manipulator joints.

```python
    nodes = [
        control_node,
       # robot_state_pub_node,
        joint_state_broadcaster_spawner,
       # delay_rviz_after_joint_state_broadcaster_spawner,
       arm_trajectory_spawner,
    ]

    return LaunchDescription(nodes)
```

- The LaunchDescription returns the list of nodes/actions launched by this file. When you run `ros2 launch zenorak_controller_actuator diffbot.launch.py` these nodes are started in the given order.

Cross references:

- `robot_description` is built from `urdf/diffbot.urdf.xacro` which contains the `<ros2_control>` plugin and joint definitions.
- `diffbot_controllers.yaml` defines the `joint_state_broadcaster` and `arm_trajectory_controller` behavior (see annotated YAML file).

Production notes and tips:

- To remap topics or pass launch arguments, you can edit this file to declare `DeclareLaunchArgument` and use `LaunchConfiguration` to apply remappings to nodes. For example, remapping `/joint_states` published by the broadcaster can be performed at the ros2_control_node level or by remapping subscriber topics downstream.
- If the controller spawner fails to start controllers, check the controller_manager logs (`ros2 topic echo /controller_manager/list_controllers`) and ensure joint names in URDF match those in controllers YAML.

# Annotated: diffbot_controllers.yaml

This file configures the controllers for the robot. The annotated explanations below describe each parameter and how it maps to the controllers started by the launcher.

```yaml
controller_manager:
  ros__parameters:
    update_rate: 10 # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    # diffbot_base_controller removed (wheel/motor interfaces removed)
    # Group trajectory controller for the 3-DOF manipulator
    arm_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController
```

- `controller_manager.ros__parameters` is used by `ros2_control_node` to pre-load controllers. The `update_rate` sets the controller manager update frequency.
- `joint_state_broadcaster` is a controller that reads state interfaces exported by the hardware plugin and republishes them on `/joint_states`.
- `arm_trajectory_controller` is a group trajectory controller that will accept trajectory goals and write position commands into the command interfaces exposed by the hardware (i.e., the `Actuator_lX.cmd` variables).

Below are the arm controller parameters explained:

```yaml
arm_trajectory_controller:
  ros__parameters:
    type: joint_trajectory_controller/JointTrajectoryController

    joints:
      - link_1_joint
      - link_2_joint
      - link_3_joint

    command_interfaces:
      - position

    state_interfaces:
      - position

    state_publish_rate: 25.0
    action_monitor_rate: 10.0
    interpolation_mode: "linear"
    allow_partial_joints_goal: false
```

- `joints`: The ordered list of joint names this controller manages. These must exactly match the joint names in the URDF that are exported under `<ros2_control>`.
- `command_interfaces` and `state_interfaces`: define the interface type(s) the controller uses. Here we only use `position` for both.
- `state_publish_rate`: how quickly the controller publishes its internal state for monitoring.
- `allow_partial_joints_goal`: whether the controller allows goals that don't include all joints.

Cross references:

- These joint names are declared in `description/ros2_control/diffbot.ros2_control.xacro` and must match `Actuator` names set in the hardware plugin.

Production notes:

- If the controller fails to activate, check that the hardware exported the expected state and command interfaces (`ros2 control list_controllers` and `ros2 topic echo /controller_manager/list_controllers`).
- Use `ros2 control` CLI to list and inspect controllers during runtime.

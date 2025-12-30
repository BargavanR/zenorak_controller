# zenorak_controller_actuator

zenorak_controller_actuator provides a ros2_control system hardware interface for a mobile robot with a differential-drive base and a 3-DOF linear-actuated manipulator.

The hardware interface communicates with a microcontroller (e.g. Arduino) over a serial connection and is designed to work with standard ROS 2 controllers such as diff_drive_controller, joint_state_broadcaster, and joint_trajectory_controller.
It is based on the diffbot example from [ros2_control demos](https://github.com/ros-controls/ros2_control_demos/tree/master/example_2).

For a tutorial on how to develop a hardware interface like this, check out the video below:

https://youtu.be/J02jEKawE5U

## To Do

# Annotated: diffbot_system.cpp

This document contains the original source lines from `hardware/diffbot_system.cpp` with friendly, line-level explanations, cross-references, and a production summary at the end.

---

```cpp
// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
```

Explanation:

- These first block comments are the standard Apache 2.0 license header. They are metadata and do not affect runtime.

```cpp
#include "zenorak_controller_actuator/diffbot_system.hpp"
```

- What it is: C++ preprocessor include that pulls in the corresponding header for this implementation file.
- Syntax: `#include "path/to/header.hpp"` includes a header from the project include path.
- Where used next: This header declares the `Zenorak_Hardware_Actuator` class and types used below.
- Called again in: The header itself is self-contained; other files may include it if they need the class declaration.
- Output: No runtime output; this line allows compilation to find declarations.

```cpp
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
```

- What it is: Standard library and ROS2 includes for time, math, containers and ROS logging.
- Syntax: `#include <...>` for system headers; `#include "..."` for project headers.
- Where used next: chrono used for sleeps and durations; rclcpp for logging and node utilities.

```cpp
namespace zenorak_controller_actuator
{
```

- What it is: Start of the package namespace. Namespaces prevent symbol collisions.

```cpp
hardware_interface::CallbackReturn Zenorak_Hardware_Actuator::on_init(
  const hardware_interface::HardwareInfo & info)
{
```

- What it is: Definition of the `on_init` method for the hardware interface class. This method is called by ros2_control when the hardware plugin is initialized.
- Syntax: C++ method definition for `Zenorak_Hardware_Actuator` class returning a `CallbackReturn`.
- Where it is called: Called by the ros2_control lifecycle when the system interface is loaded (via `controller_manager` / `ros2_control_node`).
- Output: Returns SUCCESS or ERROR to indicate initialization success.

```cpp
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
```

- What it is: Calls the base class `on_init` implementation and checks for success. If the base init fails, return an error.
- Why: Ensures basic initialization performed by the SystemInterface is honored.

```cpp
  // Device and communication parameters
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.link1_name = info_.hardware_parameters["link1_name"];
  cfg_.link2_name = info_.hardware_parameters["link2_name"];
  cfg_.link3_name = info_.hardware_parameters["link3_name"];
```

- What it is: Reads hardware parameters from the robot hardware description (ros2_control hardware parameters in the URDF/xacro). These parameters are defined in the `diffbot.ros2_control.xacro` file.
- Syntax: `info_.hardware_parameters` is a map<string,string> coming from `HardwareInfo`.
- Where it's used next: These cfg\_ fields are used in `on_configure`, `read`, and `write` to talk to the microcontroller and identify joint names.
- Called again in: see `diffbot.ros2_control.xacro` where `param name="device"` etc. are set.
- Output: No runtime output, but sets up internal configuration.

```cpp
  // Encoder/ADC counts or mapping values for the actuators
  if (info_.hardware_parameters.count("link1_enc_val") > 0) {
    cfg_.link1_enc_val = std::stoi(info_.hardware_parameters["link1_enc_val"]);
  }
  if (info_.hardware_parameters.count("link2_enc_val") > 0) {
    cfg_.link2_enc_val = std::stoi(info_.hardware_parameters["link2_enc_val"]);
  }
  if (info_.hardware_parameters.count("link3_enc_val") > 0) {
    cfg_.link3_enc_val = std::stoi(info_.hardware_parameters["link3_enc_val"]);
  }
```

- What it is: Optional parameter parsing — only overwrite cfg entries if parameters exist.
- Output: numeric mapping values used by `Actuator.setup` when converting ADC counts to angles.

```cpp
  if (info_.hardware_parameters.count("pid_p") > 0)
  {
    cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
    cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
    cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("Zenorak_Hardware_Actuator"), "PID values not supplied, using defaults.");
  }
```

- What it is: PID configuration parsing. If present, store, otherwise log info and keep defaults.
- Where used: On activation the code may send PID values to the microcontroller via `comms_.set_pid_values`.

```cpp
  // Wheels removed. Only setup actuators.
  Actuator_l1.setup(cfg_.link1_name);
  Actuator_l2.setup(cfg_.link2_name);
  Actuator_l3.setup(cfg_.link3_name);
```

- What it is: Initialize actuator helper objects with the joint names from configuration.
- Where used next: Names and conversion helpers used when exporting interfaces in `export_state_interfaces` and `export_command_interfaces`.

```cpp
  // Validate joints: we expect position state interface as first state for all joints.
  // Command interface may be velocity (for wheels) or position (for actuators).
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(...);
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto &cmd_if = joint.command_interfaces[0].name;
    if (cmd_if != hardware_interface::HW_IF_VELOCITY && cmd_if != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(...);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() < 1)
    {
      RCLCPP_FATAL(...);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(...);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() > 1 && joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(...);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
```

- What it is: Sanity checks for each joint declared in the `ros2_control` system description (URDF). Ensures the hardware-exported interfaces match expectations.
- When it's called: `on_init` receives `info_.joints` from the URDF/ros2_control description (see `diffbot.ros2_control.xacro`). If joints in URDF differ, the hardware initialization fails early with clear error logs.

```cpp
  return hardware_interface::CallbackReturn::SUCCESS;
}
```

- What it is: Signal successful initialization.

---

(continued next function definitions: `try_reconnect`, `export_state_interfaces`, `export_command_interfaces`, life-cycle methods, `read`, `write`)

Below we annotate the important runtime methods.

```cpp
void Zenorak_Hardware_Actuator::try_reconnect()
{
  static rclcpp::Time last_attempt(0, 0, RCL_ROS_TIME);
  auto now = rclcpp::Clock().now();

  if ((now - last_attempt).seconds() < 2.0)
    return;

  last_attempt = now;

  try
  {
    if (comms_.connected())
      comms_.disconnect();

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);

    RCLCPP_INFO(rclcpp::get_logger("HW"), "Actuator serial reconnected");
  }
  catch (...)
  {
    RCLCPP_WARN(rclcpp::get_logger("HW"), "Reconnect attempt failed");
  }
}
```

- What it is: Helper that tries to reconnect the serial link to the Arduino if communication is lost.
- Syntax: uses `static` to keep track of last attempt time so reconnect attempts are rate-limited to once every ~2 seconds.
- Where it's called: in `read` and `write` when `comms_.connected()` is false.
- Output: Logs success or warning messages.

```cpp
std::vector<hardware_interface::StateInterface> Zenorak_Hardware_Actuator::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Actuator position (and optional velocity) state interfaces
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    Actuator_l1.name, hardware_interface::HW_IF_POSITION, &Actuator_l1.pos));
  ...
  return state_interfaces;
}
```

- What it is: This method tells ros2_control which state interfaces (position, velocity, etc.) this hardware exposes for each joint.
- Syntax: it creates `StateInterface` objects with (joint name, interface name, pointer to internal variable).
- Where used: controller_manager uses these pointers to read positions published by hardware in `read()`.
- Output: vector of StateInterface descriptors.

```cpp
std::vector<hardware_interface::CommandInterface> Zenorak_Hardware_Actuator::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    Actuator_l1.name, hardware_interface::HW_IF_POSITION, &Actuator_l1.cmd));
  ...
  return command_interfaces;
}
```

- What it is: Exposes command interfaces (where controllers will write commands). For actuators, we accept position commands.
- Where used: controllers (e.g. `joint_trajectory_controller`) will claim these command interfaces and write into these `cmd` variables.

```cpp
hardware_interface::CallbackReturn Zenorak_Hardware_Actuator::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(..., "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  RCLCPP_INFO(..., "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}
```

- What it is: Lifecycle configure step — opens the serial port using the configured device and baud rate.
- Output: logs successful configuration. Returns SUCCESS if serial was opened successfully (or at least no exception thrown).

```cpp
hardware_interface::return_type Zenorak_Hardware_Actuator::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_.connected())
  {
  try_reconnect();
  return hardware_interface::return_type::OK;
  }

  double delta_seconds = period.seconds();

  int a1 = 0, a2 = 0, a3 = 0;
  comms_.read_actuator_values(a1, a2, a3);

  double prev_pos;

  // Actuator 1
  prev_pos = Actuator_l1.pos;
  Actuator_l1.raw_count = a1;
  Actuator_l1.pos = Actuator_l1.potToAngle(
    a1,
    0.0, 80.0,     // logical angle range
    460, 730       // pot range
  );
  Actuator_l1.vel = (Actuator_l1.pos - prev_pos) / delta_seconds;

  ... (actuator 2 and 3 handled similarly)

  RCLCPP_INFO(..., "Arm pos: %f %f %f", Actuator_l1.pos, Actuator_l2.pos, Actuator_l3.pos);

  return hardware_interface::return_type::OK;
}
```

- What it is: Called periodically by controller_manager to fetch the latest hardware state and update the exported state interface variables.
- Input: `period` gives the time since last call. Used to compute velocities.
- Steps:
  - If serial not connected, try reconnect and return OK (no state change).
  - Read raw actuator counts from Arduino via `comms_`.
  - Convert raw counts to logical joint angles using `Actuator::potToAngle`.
  - Compute velocities based on position delta and `period`.
- Output: updates `Actuator_lX.pos` and `.vel` in-place (these are the state interface targets that controllers or broadcasters read).
- Called by: controller_manager main loop.

```cpp
hardware_interface::return_type zenorak_controller_actuator ::Zenorak_Hardware_Actuator::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
  try_reconnect();
  return hardware_interface::return_type::OK;
  }

  int a1_target = Actuator_l1.angleToPot(
    Actuator_l1.cmd,
    0.0, 80.0,     // angle range in degrees
    460, 730        // pot range
  );
  ... (a2_target, a3_target)

  comms_.set_actuator_positions(a1_target, a2_target, a3_target);
  return hardware_interface::return_type::OK;
}
```

- What it is: Called periodically and sends the latest command values to the microcontroller.
- Input: `Actuator_lX.cmd` is written by the controller that claims the `CommandInterface` (e.g. `joint_trajectory_controller`).
- Output: Sends an `s v1 v2 v3\r` message to the Arduino via `ArduinoComms::set_actuator_positions`.

---

Where functions/variables are used across the package (quick cross-reference):

- `cfg_.device`, `cfg_.baud_rate`, `cfg_.timeout_ms` are set in `on_init` and used in `on_configure` to open serial in `comms_.connect`.
- `Actuator_lX` objects are declared in `diffbot_system.hpp` and their methods are defined in `Actuator.hpp` (see annotated file for the Actuator class).
- `comms_.read_actuator_values` and `comms_.set_actuator_positions` are defined in `arduino_comms.hpp`.
- The hardware plugin is referenced in `description/ros2_control/diffbot.ros2_control.xacro` via the `<plugin>` tag.

Production summary (child-friendly):

- This file is the bridge between ROS controllers and the robot hardware (Arduino). It reads sensor values (potentiometer counts) and tells ROS what the joint angles are, and it receives position commands from controllers and sends them to the Arduino. It checks and opens the serial port, converts raw counts into angles, and keeps everything synchronized with the ROS controller loop.

If you'd like, I can now create the same kind of annotated file for the other headers and launch/config files and then write a friendly, step-by-step README explaining how to build, run, and integrate this package into URDF/xacro with ros2_control.

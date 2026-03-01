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

#include "zenorak_controller_motor/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// This file implements the hardware interface that connects ros2_control to
// the physical motors mounted on the diffbot. It exposes two wheel joints
// (left and right) via the standard ros2_control state/command interfaces.
// High-level controllers (e.g. diff_drive_controller) will talk to these
// interfaces to command wheel velocities.

namespace zenorak_controller_motor
{
// on_init: called by the ros2_control framework during initialization.
// It receives the hardware info parsed from the robot's ros2_control
// hardware block (typically supplied via an xacro/URDF). Here we read
// parameters such as serial device, baud rate and joint names and then
// perform minimal initialization of helper objects.
hardware_interface::CallbackReturn Zenorak_Hardware_Motor::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // ensure base class initialization succeeded
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read configuration values from the <hardware> section in your URDF/xacro
  // Example keys available in info_.hardware_parameters: left_wheel_name, right_wheel_name, loop_rate, device, baud_rate, timeout_ms, enc_counts_per_rev
  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  
  // Initialize wheel helpers with joint names and encoder resolution
  wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  // Validate joints provided in the hardware info. ros2_control expects the first
  // state interface to be position; command interface can be velocity (for wheels).
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Zenorak_Hardware_Motor"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto &cmd_if = joint.command_interfaces[0].name;
    if (cmd_if != hardware_interface::HW_IF_VELOCITY && cmd_if != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Zenorak_Hardware_Motor"),
        "Joint '%s' has unsupported command interface '%s'. '%s' or '%s' expected.",
        joint.name.c_str(), cmd_if.c_str(), hardware_interface::HW_IF_VELOCITY,
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() < 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Zenorak_Hardware_Motor"),
        "Joint '%s' has %zu state interfaces. At least 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Zenorak_Hardware_Motor"),
        "Joint '%s' has '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() > 1 && joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Zenorak_Hardware_Motor"),
        "Joint '%s' has unexpected second state interface '%s'. '%s' expected if present.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}



// try_reconnect: attempt to reconnect serial port at most once every ~2 seconds.
void Zenorak_Hardware_Motor::try_reconnect()
{
  static rclcpp::Time last_attempt(0, 0, RCL_ROS_TIME);
  auto now = rclcpp::Clock().now();

  // rate-limit reconnect attempts
  if ((now - last_attempt).seconds() < 2.0)
    return;

  last_attempt = now;

  try
  {
    if (comms_.connected())
      comms_.disconnect();

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);

    RCLCPP_INFO(rclcpp::get_logger("HW"), "motor serial reconnected");
  }
  catch (...)
  {
    RCLCPP_WARN(rclcpp::get_logger("HW"), "Reconnect attempt failed");
  }
}

// export_state_interfaces: expose position/velocity state for both wheels
std::vector<hardware_interface::StateInterface> Zenorak_Hardware_Motor::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Each StateInterface takes: joint name, interface name (position/velocity), pointer to the variable
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));
  return state_interfaces;
}

// export_command_interfaces: expose velocity command interfaces for both wheels
std::vector<hardware_interface::CommandInterface> Zenorak_Hardware_Motor::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // CommandInterface takes: joint name, interface name, pointer to command variable
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));
  return command_interfaces;
}

// on_configure: lifecycle transition - open serial port
hardware_interface::CallbackReturn Zenorak_Hardware_Motor::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Zenorak_Hardware_Motor"), "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  RCLCPP_INFO(rclcpp::get_logger("Zenorak_Hardware_Motor"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// on_cleanup: lifecycle transition - close serial port
hardware_interface::CallbackReturn Zenorak_Hardware_Motor::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Zenorak_Hardware_Motor"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("Zenorak_Hardware_Motor"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


// on_activate: lifecycle transition - ensure serial is connected
hardware_interface::CallbackReturn Zenorak_Hardware_Motor::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Zenorak_Hardware_Motor"), "Activating ...please wait...");
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  // nothing else to do for activation in the motor-only variant
  RCLCPP_INFO(rclcpp::get_logger("Zenorak_Hardware_Motor"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// on_deactivate: lifecycle transition - nothing special to do
hardware_interface::CallbackReturn Zenorak_Hardware_Motor::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Zenorak_Hardware_Motor"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("Zenorak_Hardware_Motor"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// read: core real-time-safe-ish read function called by ros2_control.
// It should read sensors (encoders) and update the state variables.
hardware_interface::return_type Zenorak_Hardware_Motor::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_.connected())
  {
  try_reconnect();
  return hardware_interface::return_type::OK;
  }

  // Read encoder counts from microcontroller and update wheel state
  comms_.read_encoder_values(wheel_l_.enc, wheel_r_.enc);

  double delta_seconds = period.seconds();

  double pos_prev = wheel_l_.pos;
  wheel_l_.pos = wheel_l_.calc_enc_angle();
  wheel_l_.vel = (wheel_l_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_r_.pos;
  wheel_r_.pos = wheel_r_.calc_enc_angle();
  wheel_r_.vel = (wheel_r_.pos - pos_prev) / delta_seconds;
  return hardware_interface::return_type::OK;
}

// write: convert velocity commands into motor counts-per-loop and send them
hardware_interface::return_type zenorak_controller_motor ::Zenorak_Hardware_Motor::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
  try_reconnect();
  return hardware_interface::return_type::OK;
  }

  // Convert commanded velocity (rad/s) to counts-per-loop expected by MCU.
  // Formula: counts_per_loop = cmd_rad_per_sec / rads_per_count / loop_rate
  int motor_l_counts_per_loop = wheel_l_.cmd / wheel_l_.rads_per_count / cfg_.loop_rate;
  int motor_r_counts_per_loop = wheel_r_.cmd / wheel_r_.rads_per_count / cfg_.loop_rate;
  comms_.set_motor_values(motor_l_counts_per_loop, motor_r_counts_per_loop);

  // Actuator support removed — only motor values are sent
  return hardware_interface::return_type::OK;
}

}  // namespace zenorak_controller_motor

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  zenorak_controller_motor::Zenorak_Hardware_Motor, hardware_interface::SystemInterface)

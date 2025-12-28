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

#include "zenorak_controller/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace zenorak_controller
{
hardware_interface::CallbackReturn Zenorak_Hardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }


  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  cfg_.link1_name = info_.hardware_parameters["link1_name"];
  cfg_.link2_name = info_.hardware_parameters["link2_name"];
  cfg_.link3_name = info_.hardware_parameters["link3_name"];
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
  if (info_.hardware_parameters.count("pid_p") > 0)
  {
    cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
    cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
    cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("Zenorak_Hardware"), "PID values not supplied, using defaults.");
  }
  

  wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);
  Actuator_l1.setup(cfg_.link1_name);
  Actuator_l2.setup(cfg_.link2_name);
  Actuator_l3.setup(cfg_.link3_name);

  // Validate joints: we expect position state interface as first state for all joints.
  // Command interface may be velocity (for wheels) or position (for actuators).
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Zenorak_Hardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto &cmd_if = joint.command_interfaces[0].name;
    if (cmd_if != hardware_interface::HW_IF_VELOCITY && cmd_if != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Zenorak_Hardware"),
        "Joint '%s' has unsupported command interface '%s'. '%s' or '%s' expected.",
        joint.name.c_str(), cmd_if.c_str(), hardware_interface::HW_IF_VELOCITY,
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() < 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Zenorak_Hardware"),
        "Joint '%s' has %zu state interfaces. At least 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Zenorak_Hardware"),
        "Joint '%s' has '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() > 1 && joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Zenorak_Hardware"),
        "Joint '%s' has unexpected second state interface '%s'. '%s' expected if present.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Zenorak_Hardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  // Actuator position (and optional velocity) state interfaces
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    Actuator_l1.name, hardware_interface::HW_IF_POSITION, &Actuator_l1.pos));
  // Note: do NOT export velocity state for actuators — only position is required/used.

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    Actuator_l2.name, hardware_interface::HW_IF_POSITION, &Actuator_l2.pos));
  // velocity intentionally not exported for actuator 2

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    Actuator_l3.name, hardware_interface::HW_IF_POSITION, &Actuator_l3.pos));
  // velocity intentionally not exported for actuator 3

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Zenorak_Hardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  // Actuators accept position commands (joint_trajectory_controller will send position commands)
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    Actuator_l1.name, hardware_interface::HW_IF_POSITION, &Actuator_l1.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    Actuator_l2.name, hardware_interface::HW_IF_POSITION, &Actuator_l2.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    Actuator_l3.name, hardware_interface::HW_IF_POSITION, &Actuator_l3.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn Zenorak_Hardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Zenorak_Hardware"), "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  RCLCPP_INFO(rclcpp::get_logger("Zenorak_Hardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Zenorak_Hardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Zenorak_Hardware"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("Zenorak_Hardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn Zenorak_Hardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Zenorak_Hardware"), "Activating ...please wait...");
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (cfg_.pid_p > 0)
  {
    comms_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
  }
  RCLCPP_INFO(rclcpp::get_logger("Zenorak_Hardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Zenorak_Hardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Zenorak_Hardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("Zenorak_Hardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Zenorak_Hardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  comms_.read_encoder_values(wheel_l_.enc, wheel_r_.enc);

  double delta_seconds = period.seconds();

  double pos_prev = wheel_l_.pos;
  wheel_l_.pos = wheel_l_.calc_enc_angle();
  wheel_l_.vel = (wheel_l_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_r_.pos;
  wheel_r_.pos = wheel_r_.calc_enc_angle();
  wheel_r_.vel = (wheel_r_.pos - pos_prev) / delta_seconds;

  // Read actuator raw counts/ADC values from Arduino and update actuator state
  int a1 = 0, a2 = 0, a3 = 0;
  comms_.read_actuator_values(a1, a2, a3);
  
  double prev_pos;

  // Actuator 1
  prev_pos = Actuator_l1.pos;
  Actuator_l1.raw_count = a1;
  Actuator_l1.pos = Actuator_l1.potToAngle(
    a1,
    0.0, 80.0,     // logical angle range
    470, 670       // pot range
  );
  Actuator_l1.vel = (Actuator_l1.pos - prev_pos) / delta_seconds;

  // Actuator 2
  prev_pos = Actuator_l2.pos;
  Actuator_l2.raw_count = a2;
  Actuator_l2.pos = Actuator_l2.potToAngle(
    a2,
    0.0, 80.0,
    150, 430
  );
  Actuator_l2.vel = (Actuator_l2.pos - prev_pos) / delta_seconds;

  // Actuator 3
  prev_pos = Actuator_l3.pos;
  Actuator_l3.raw_count = a3;
  Actuator_l3.pos = Actuator_l3.potToAngle(
    a3,
    0.0, 80.0,
    90, 460
  );
  Actuator_l3.vel = (Actuator_l3.pos - prev_pos) / delta_seconds;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type zenorak_controller ::Zenorak_Hardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  int motor_l_counts_per_loop = wheel_l_.cmd / wheel_l_.rads_per_count / cfg_.loop_rate;
  int motor_r_counts_per_loop = wheel_r_.cmd / wheel_r_.rads_per_count / cfg_.loop_rate;
  comms_.set_motor_values(motor_l_counts_per_loop, motor_r_counts_per_loop);

  // Convert actuator desired positions (radians) into counts and send to Arduino
  int a1_target = Actuator_l1.angleToPot(
    Actuator_l1.cmd,
    0.0, 80.0,     // angle range in degrees
    470, 670         // pot range
  );

  int a2_target = Actuator_l2.angleToPot(
    Actuator_l2.cmd,
    0.0, 80.0,
    150, 430
  );

  int a3_target = Actuator_l3.angleToPot(
    Actuator_l3.cmd,
    0.0, 80.0,
    90, 460
  );
  static double last_a1 = 470, last_a2 = 150, last_a3 = 90;

  if (a1_target != last_a1 ||
      a1_target != last_a2 ||
      a1_target != last_a3)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("HW"),
      "Arm cmd: %d %d %d | Wheels: L=%d R=%d",
      a1_target,
      a2_target,
      a3_target,
      motor_l_counts_per_loop,
      motor_r_counts_per_loop
    );

    last_a1 = a1_target;
    last_a2 = a2_target;
    last_a3 = a3_target;
  }

  comms_.set_actuator_positions(a1_target, a2_target, a3_target);
  return hardware_interface::return_type::OK;
}

}  // namespace zenorak_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  zenorak_controller::Zenorak_Hardware, hardware_interface::SystemInterface)

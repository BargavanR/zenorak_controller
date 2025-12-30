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

#ifndef ZENORAK_CONTROLLER_MOTOR__DIFFBOT_SYSTEM_HPP_
#define ZENORAK_CONTROLLER_MOTOR__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "zenorak_controller_motor/visibility_control.h"

// Hardware-specific helpers used by this SystemInterface implementation
// ArduinoComms: serial/Arduino communication helpers
// Wheel: simple POD storing encoder, command and state
#include "zenorak_controller_motor/arduino_comms.hpp"
#include "zenorak_controller_motor/wheel.hpp"


namespace zenorak_controller_motor
{
class Zenorak_Hardware_Motor : public hardware_interface::SystemInterface
{

struct Config
{
  std::string left_wheel_name = "";
  std::string right_wheel_name = "";
  float loop_rate = 0.0;
  std::string device = "";
  int baud_rate = 0;
  int timeout_ms = 0;
  int enc_counts_per_rev = 0;
  // Notes:
  // - left_wheel_name/right_wheel_name: names of the joint entries in your URDF/xacro
  // - loop_rate: control loop frequency used to convert velocity to counts-per-loop
  // - device: serial device path (eg. /dev/ttyACM0)
  // - baud_rate / timeout_ms: serial settings used by ArduinoComms
  // - enc_counts_per_rev: encoder resolution used to compute radians from counts

};


public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Zenorak_Hardware_Motor);

  ZENORAK_CONTROLLER_MOTOR_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  ZENORAK_CONTROLLER_MOTOR_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ZENORAK_CONTROLLER_MOTOR_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ZENORAK_CONTROLLER_MOTOR_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ZENORAK_CONTROLLER_MOTOR_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;


  ZENORAK_CONTROLLER_MOTOR_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ZENORAK_CONTROLLER_MOTOR_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ZENORAK_CONTROLLER_MOTOR_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ZENORAK_CONTROLLER_MOTOR_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  ArduinoComms comms_;
  Config cfg_;
  Wheel wheel_l_;
  Wheel wheel_r_;
  // try_reconnect: convenience method used when the serial connection is lost.
  // It attempts to reconnect the serial port every couple of seconds.
  void try_reconnect();

};

}  // namespace zenorak_controller_motor

#endif  // ZENORAK_CONTROLLER_MOTOR__DIFFBOT_SYSTEM_HPP_

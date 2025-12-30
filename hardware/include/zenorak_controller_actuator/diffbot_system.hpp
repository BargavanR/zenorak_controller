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

#ifndef ZENORAK_CONTROLLER_ACTUATOR__DIFFBOT_SYSTEM_HPP_
#define ZENORAK_CONTROLLER_ACTUATOR__DIFFBOT_SYSTEM_HPP_

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
#include "zenorak_controller_actuator/visibility_control.h"

#include "zenorak_controller_actuator/arduino_comms.hpp"
#include "zenorak_controller_actuator/Actuator.hpp"


namespace zenorak_controller_actuator
{
class Zenorak_Hardware_Actuator : public hardware_interface::SystemInterface
{

struct Config
{
  // Wheel/motor parameters removed
  std::string device = "";
  int baud_rate = 0;
  int timeout_ms = 0;
  std::string link1_name = "";
  std::string link2_name = "";
  std::string link3_name = "";
  int link1_enc_val = 0;
  int link2_enc_val = 0;
  int link3_enc_val = 0;
  int pid_p = 0;
  int pid_d = 0;
  int pid_i = 0;
  int pid_o = 0;
};


public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Zenorak_Hardware_Actuator);

  ZENORAK_CONTROLLER_ACTUATOR_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  ZENORAK_CONTROLLER_ACTUATOR_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ZENORAK_CONTROLLER_ACTUATOR_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ZENORAK_CONTROLLER_ACTUATOR_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ZENORAK_CONTROLLER_ACTUATOR_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;


  ZENORAK_CONTROLLER_ACTUATOR_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ZENORAK_CONTROLLER_ACTUATOR_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ZENORAK_CONTROLLER_ACTUATOR_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ZENORAK_CONTROLLER_ACTUATOR_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  ArduinoComms comms_;
  Config cfg_;
  // Wheels/motors removed - only actuators handled now
  Actuator Actuator_l1;
  Actuator Actuator_l2;
  Actuator Actuator_l3;
  void try_reconnect();

};

}  // namespace zenorak_controller_actuator

#endif  // ZENORAK_CONTROLLER_ACTUATOR__DIFFBOT_SYSTEM_HPP_

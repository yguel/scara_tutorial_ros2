// Copyright 2022, ICube Laboratory, University of Strasbourg
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

#include "scara_hardware/scara_robot.hpp"
#include "rclcpp/rclcpp.hpp"

namespace scara_hardware
{
// ------------------------------------------------------------------------------------------
CallbackReturn ScaraRobot::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  // Allocate memory
  hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_previous_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Check description compatibility
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // scaraRobot has currently exactly 2 state and 1 command interface on each joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger("ScaraRobot"),
        "Joint '%s' has %li command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("ScaraRobot"),
          "Joint '%s' has %s command interfaces. '%s' expected", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return CallbackReturn::ERROR;
      }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(
        rclcpp::get_logger("ScaraRobot"),
        "Joint '%s' has %li state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger("ScaraRobot"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger("ScaraRobot"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  // initialize states
  for (uint i = 0; i < info_.joints.size(); i++) {
    hw_states_position_[i] = std::stod(info_.joints[i].state_interfaces[0].initial_value);
    hw_states_previous_position_[i] = std::stod(info_.joints[i].state_interfaces[0].initial_value);
    hw_states_velocity_[i] = std::stod(info_.joints[i].state_interfaces[1].initial_value);
  }
  return CallbackReturn::SUCCESS;
}
// ------------------------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
ScaraRobot::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));

  }
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));

  }

  return state_interfaces;
}
// ------------------------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
ScaraRobot::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]));
  }

  return command_interfaces;
}
// ------------------------------------------------------------------------------------------
hardware_interface::return_type ScaraRobot::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{
  for (uint i = 0; i < info_.joints.size(); i++) {
    hw_states_velocity_[i] = (hw_states_position_[i] - hw_states_previous_position_[i])/(period.nanoseconds()*1e-9);

    hw_states_previous_position_[i] = hw_states_position_[i];
  }

  return hardware_interface::return_type::OK;
}
// ------------------------------------------------------------------------------------------
hardware_interface::return_type ScaraRobot::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // write hw_commands_position_ 
  bool isNan = false;
  for (auto i = 0ul; i < hw_commands_position_.size(); i++) {
    if (hw_commands_position_[i] != hw_commands_position_[i]) {
      isNan = true;
    }
  }

  if (!isNan) {
    for (uint i = 0; i < info_.joints.size(); i++) {
      double min_position = std::stod(info_.joints[i].state_interfaces[0].min);
      double max_position = std::stod(info_.joints[i].state_interfaces[0].max);

      hw_states_position_[i] = hw_commands_position_[i];

      if(hw_states_position_[i] > max_position) hw_states_position_[i] = max_position;
      if(hw_states_position_[i] < min_position) hw_states_position_[i] = min_position;
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace scara_hardware
// ---------------------------------------------------------------------------
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  scara_hardware::ScaraRobot, hardware_interface::SystemInterface)
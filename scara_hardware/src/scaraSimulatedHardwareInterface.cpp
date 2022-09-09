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

#include "scara_hardware/scaraSimulatedHardwareInterface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace scara_hardware
{
// ------------------------------------------------------------------------------------------
CallbackReturn ScaraSimulatedHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_previous_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // scaraRobot has currently exactly 3 state and 1 command interface on each joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger("ScaraSimulatedHardwareInterface"),
        "Joint '%s' has %li command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("ScaraSimulatedHardwareInterface"),
          "Joint '%s' has %s command interfaces. '%s' expected", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return CallbackReturn::ERROR;
      }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(
        rclcpp::get_logger("ScaraSimulatedHardwareInterface"),
        "Joint '%s' has %li state interface. 3 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger("ScaraSimulatedHardwareInterface"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger("ScaraSimulatedHardwareInterface"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(
        rclcpp::get_logger("ScaraSimulatedHardwareInterface"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }
  }
  return CallbackReturn::SUCCESS;
}
// ------------------------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
ScaraSimulatedHardwareInterface::export_state_interfaces()
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
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_effort_[i]));

  }

  return state_interfaces;
}
// ------------------------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
ScaraSimulatedHardwareInterface::export_command_interfaces()
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
CallbackReturn ScaraSimulatedHardwareInterface::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(rclcpp::get_logger("ScaraSimulatedHardwareInterface"), "Starting ...please wait...");

  for (uint i = 0; i < info_.joints.size(); i++) {
    hw_states_position_[i] = 0;
    hw_states_previous_position_[i] = 0;
    hw_states_velocity_[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("ScaraSimulatedHardwareInterface"), "System Successfully started!");

  return CallbackReturn::SUCCESS;
}
// ------------------------------------------------------------------------------------------
CallbackReturn ScaraSimulatedHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(rclcpp::get_logger("ScaraSimulatedHardwareInterface"), "Stopping ...please wait...");

  RCLCPP_INFO(
    rclcpp::get_logger("ScaraSimulatedHardwareInterface"), "System successfully stopped!");

  return CallbackReturn::SUCCESS;
}
// ------------------------------------------------------------------------------------------
hardware_interface::return_type ScaraSimulatedHardwareInterface::read(
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
hardware_interface::return_type ScaraSimulatedHardwareInterface::write(
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
        hw_states_position_[i] = hw_commands_position_[i];
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace scara_hardware
// ---------------------------------------------------------------------------
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  scara_hardware::ScaraSimulatedHardwareInterface, hardware_interface::SystemInterface)
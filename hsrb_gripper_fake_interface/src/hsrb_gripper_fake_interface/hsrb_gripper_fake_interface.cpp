/*
Copyright (c) 2025 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#include <algorithm>
#include <memory>
#include <utility>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "hsrb_gripper_fake_interface.hpp"


namespace {
constexpr double kMinFingerAngle = -7.0 * M_PI / 180.0;
constexpr double kMaxFingerAngle = 70.0 * M_PI / 180.0;
}  // namespace

namespace hsrb_gripper_fake_interface {

CallbackReturn HsrbGripperFakeInterface::on_init(const hardware_interface::HardwareInfo& hardware_info) {
  // Execute the initialization process of the parent class
  if (SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  // Retrieve the joint name of the hand described in ros2_control.rviz.xacro
  joint_name_ = hardware_info.joints[0].name;

  // Initialize various variables
  command_position_ = 0.0;
  command_effort_ = 0.0;
  command_drive_mode_ = -1.0;
  command_grasping_flag_ = 0.0;

  previous_command_position_ = 0.0;

  state_position_ = 0.0;
  state_velocity_ = 0.0;
  state_effort_ = 0.0;
  state_drive_mode_ = 0.0;
  state_grasping_flag_ = 0.0;

  last_velocity_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // Retrieve the threshold parameter for position
  if (hardware_info.hardware_parameters.find("position_min") != hardware_info.hardware_parameters.end()) {
    param_position_min_ = std::stod(hardware_info.hardware_parameters.at("position_min"));
  } else {
    param_position_min_ = kMinFingerAngle;
  }
  if (hardware_info.hardware_parameters.find("position_max") != hardware_info.hardware_parameters.end()) {
    param_position_max_ = std::stod(hardware_info.hardware_parameters.at("position_max"));
  } else {
    param_position_max_ = kMaxFingerAngle;
  }
  RCLCPP_INFO(rclcpp::get_logger(hardware_info.name),
              "Position min: %f, Position max: %f", param_position_min_, param_position_max_);

  // Retrieve the threshold parameter for effort
  if (hardware_info.hardware_parameters.find("effort_min") != hardware_info.hardware_parameters.end()) {
    param_effort_min_ = std::stod(hardware_info.hardware_parameters.at("effort_min"));
  } else {
     // If the parameter does not exist, use the threshold 0.011, which is the measured value of unit 34
    param_effort_min_ = 0.011;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HsrbGripperFakeInterface::export_state_interfaces() {
  // Add information for each state
  std::vector<hardware_interface::StateInterface> state_interfaces;
  // Add position information
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(joint_name_, hardware_interface::HW_IF_POSITION, &state_position_));

  // Add velocity information
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(joint_name_, hardware_interface::HW_IF_VELOCITY, &state_velocity_));

  // Add effort information
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(joint_name_, hardware_interface::HW_IF_EFFORT, &state_effort_));

  // Add drive mode information
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(joint_name_, "current_drive_mode", &state_drive_mode_));

  // Add grasping flag information
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(joint_name_, "current_grasping_flag", &state_grasping_flag_));

  // Add current information
  state_interfaces.emplace_back(hardware_interface::StateInterface(joint_name_, "current", &state_current_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HsrbGripperFakeInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Add position information
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(joint_name_, hardware_interface::HW_IF_POSITION, &command_position_));

  // Add effort information
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(joint_name_, hardware_interface::HW_IF_EFFORT, &command_effort_));

  // Add drive mode information
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(joint_name_, "command_drive_mode", &command_drive_mode_));

  // Add grasping flag information
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(joint_name_, "command_grasping_flag", &command_grasping_flag_));

  return command_interfaces;
}

CallbackReturn HsrbGripperFakeInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

CallbackReturn HsrbGripperFakeInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

CallbackReturn HsrbGripperFakeInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type HsrbGripperFakeInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HsrbGripperFakeInterface::write(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
  // Retrieve the instruction for the grasp flag and the current state
  const auto command_grasping_flag = static_cast<int>(command_grasping_flag_);
  const auto state_grasping_flag = static_cast<int>(state_grasping_flag_);

  // When there is a change in the grasp flag
  if (command_grasping_flag != state_grasping_flag) {
    // When the grasp flag is 1
    if (command_grasping_flag == 1) {
      // The operation is as follows based on the effort value
      // 1. The absolute value of effort is greater than the threshold
      //  1.1. When the effort value is positive: fully open
      //  1.2. When the effort value is negative: fully closed
      if (fabs(command_effort_) > param_effort_min_) {
        if (command_effort_ > 0) {
          state_position_ = 1.0;
        } else {
          state_position_ = 0.0;
        }
      }
    }
    // Update the state of grasping
    state_effort_ = command_effort_;
    state_grasping_flag_ = command_grasping_flag_;
  }

  // If there is a difference in value from the previous cycle, update the position value
  if (previous_command_position_ != command_position_) {
    state_position_ = std::max(param_position_min_, std::min(param_position_max_, command_position_));

    // Calculate velocity at the timing of position update
    state_velocity_ = (command_position_ - previous_command_position_)
                      / (time - last_velocity_update_time_).seconds();
    last_velocity_update_time_ = time;
  }
  previous_command_position_ = command_position_;

  // Set velocity to 0 if it has not been updated for a certain period of time
  if ((time - last_velocity_update_time_).seconds() > 1.0) {
    state_velocity_ = 0.0;
  }

  // Execute the update of drive mode
  // However, since the simulator side does not have drive_mode settings, only update the status
  const auto command_drive_mode = static_cast<int>(command_drive_mode_);
  if (command_drive_mode >= 0) {
    state_drive_mode_ = command_drive_mode_;
    command_drive_mode_ = -1.0;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace hsrb_gripper_fake_interface


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(hsrb_gripper_fake_interface::HsrbGripperFakeInterface, hardware_interface::SystemInterface)

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
/// @file hsrb_gripper_fake_interface.hpp
/// HsrbGripperFakeInterface class
#ifndef HSRB_GRIPPER_FAKE_INTERFACE_HPP_
#define HSRB_GRIPPER_FAKE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hsrb_gripper_fake_interface {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HsrbGripperFakeInterface : public hardware_interface::SystemInterface {
 public:
  virtual ~HsrbGripperFakeInterface() = default;

  CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string>& /*start_interfaces*/,
      const std::vector<std::string>& /*stop_interfaces*/) override {
    return hardware_interface::return_type::OK;
  }

 protected:
  // hand joint name
  std::string joint_name_;

  // current position
  double state_position_;
  // current velocity
  double state_velocity_;
  // current effort
  double state_effort_;
  // current drive mode
  double state_drive_mode_;
  // current grasping flag
  double state_grasping_flag_;
  // current value
  double state_current_;

  // position command value
  double command_position_;
  // effort command value
  double command_effort_;
  // drive mode command value
  double command_drive_mode_;
  // grasping flag command value
  double command_grasping_flag_;

  // previous cycle position command value
  double previous_command_position_;

  // minimum position value
  double param_position_min_;
  // maximum position value
  double param_position_max_;
  // effort threshold
  double param_effort_min_;

  // last time velocity was updated
  rclcpp::Time last_velocity_update_time_;
};

}  // namespace hsrb_gripper_fake_interface

#endif /* HSRB_GRIPPER_FAKE_INTERFACE_HPP_ */

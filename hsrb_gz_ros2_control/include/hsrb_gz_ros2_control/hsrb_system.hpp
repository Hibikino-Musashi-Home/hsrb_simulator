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
#ifndef HSRB_GZ_ROS2_CONTROL__GZ_SYSTEM_HPP_
#define HSRB_GZ_ROS2_CONTROL__GZ_SYSTEM_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <gz_ros2_control/gz_system_interface.hpp>

#include <pluginlib/class_loader.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <tmc_exxx_servo_motor_protocol/exxx_common.hpp>

namespace hsrb_gz_ros2_control {

struct JointData {
  /// \brief Joint's names.
  std::string name;

  /// \brief Current joint position
  double joint_position;

  /// \brief Current joint velocity
  double joint_velocity;

  /// \brief Current joint effort
  double joint_effort;

  /// \brief Current cmd joint position
  double joint_position_cmd;

  /// \brief Current cmd joint velocity
  double joint_velocity_cmd;

  /// \brief Current cmd joint effort
  double joint_effort_cmd;

  /// \brief flag if joint is actuated (has command interfaces) or passive
  bool is_actuated;

  /// \brief handles to the joints from within Gazebo
  sim::Entity sim_joint;

  /// \brief Control method defined in the URDF for each joint.
  gz_ros2_control::GazeboSimSystemInterface::ControlMethod joint_control_method;
};

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class GazeboSimSystem : public gz_ros2_control::GazeboSimSystemInterface {
 public:
  GazeboSimSystem() :
    gz_system_loader_("gz_ros2_control", "gz_ros2_control::GazeboSimSystemInterface"),
    hand_l_spring_proximal_joint_pos_(0.0),
    hand_r_spring_proximal_joint_pos_(0.0),
    hand_l_spring_proximal_joint_vel_(0.0),
    hand_r_spring_proximal_joint_vel_(0.0),
    hand_spring_coeff_(1.0),
    drive_mode_(tmc_exxx_servo_motor_protocol::kDriveModeNoControl),
    drive_mode_cmd_(tmc_exxx_servo_motor_protocol::kDriveModeNoControl),
    grasping_flag_(0.0),
    grasping_flag_cmd_(0.0) {}

  CallbackReturn on_init(const hardware_interface::HardwareInfo& system_info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;

  hardware_interface::return_type read(
      const rclcpp::Time& time,
      const rclcpp::Duration& period) override;

  hardware_interface::return_type write(
      const rclcpp::Time& time,
      const rclcpp::Duration& period) override;

  bool initSim(
      rclcpp::Node::SharedPtr& model_nh,
      std::map<std::string, sim::Entity>& joints,
      const hardware_interface::HardwareInfo& hardware_info,
      sim::EntityComponentManager& _ecm,
      int& update_rate) override;

  std::string get_name() const override { return this->parent_->get_name(); }

 private:
  std::vector<hardware_interface::StateInterface> state_interfaces_;
  std::vector<hardware_interface::CommandInterface> command_interfaces_;

  // Accessing the parent's struct is impossible, so intercepting the interface to retrieve and modify necessary information
  std::vector<hardware_interface::CommandInterface> parent_motor_position_command_;
  std::vector<hardware_interface::StateInterface> parent_motor_position_state_;

  std::unique_ptr<gz_ros2_control::GazeboSimSystemInterface> parent_;
  pluginlib::ClassLoader<gz_ros2_control::GazeboSimSystemInterface> gz_system_loader_;

  sim::EntityComponentManager* ecm_{nullptr};

  JointData motor_joint_;

  sim::Entity hand_l_spring_proximal_joint_;
  sim::Entity hand_r_spring_proximal_joint_;
  double hand_l_spring_proximal_joint_pos_;
  double hand_r_spring_proximal_joint_pos_;
  double hand_l_spring_proximal_joint_vel_;
  double hand_r_spring_proximal_joint_vel_;

  struct JointSaturation {
    sim::Entity joint;
    double lower;
    double upper;
  };
  std::vector<JointSaturation> joint_saturations_;

  // spring coefficient [Nm/rad]
  double hand_spring_coeff_;

  double grasp_velocity_gain_;
  double grasp_effort_tolerance_;

  double drive_mode_;
  double drive_mode_cmd_;
  double grasping_flag_;
  double grasping_flag_cmd_;
  double current_;
};

}  // namespace hsrb_gz_ros2_control
#endif  // HSRB_GZ_ROS2_CONTROL__GZ_SYSTEM_HPP_

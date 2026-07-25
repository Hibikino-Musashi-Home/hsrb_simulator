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
#include "hsrb_gz_ros2_control/hsrb_system.hpp"

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointForce.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#define GZ_TRANSPORT_NAMESPACE gz::transport::
#define GZ_MSGS_NAMESPACE gz::msgs::

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace {

// Speed gain of the gripping motion
constexpr double kGraspVelocityGain = 100.0;
// Torque tolerance error of the gripping motion [Nm]
constexpr double kGraspEffortTolerance = 0.01;

}  // unnamed namespace


namespace hsrb_gz_ros2_control {

void GazeboSimGripperSystem::read(const sim::EntityComponentManager* ecm) {
  if (!(this->state_interface_)) {
    return;
  }
  this->motor_joint_.joint_position = state_interface_->get_value();

  this->hand_l_spring_proximal_joint_pos_ =
      ecm->Component<sim::components::JointPosition>(this->hand_l_spring_proximal_joint_)->Data()[0];
  this->hand_r_spring_proximal_joint_pos_ =
      ecm->Component<sim::components::JointPosition>(this->hand_r_spring_proximal_joint_)->Data()[0];
  this->hand_l_spring_proximal_joint_vel_ =
      ecm->Component<sim::components::JointVelocity>(this->hand_l_spring_proximal_joint_)->Data()[0];
  this->hand_r_spring_proximal_joint_vel_ =
      ecm->Component<sim::components::JointVelocity>(this->hand_r_spring_proximal_joint_)->Data()[0];

  const double hand_l_torque = this->hand_l_spring_proximal_joint_pos_ * this->hand_spring_coeff_;
  const double hand_r_torque = this->hand_r_spring_proximal_joint_pos_ * this->hand_spring_coeff_;
  this->motor_joint_.joint_effort = -(hand_l_torque + hand_r_torque) / 2.0;
}

void GazeboSimGripperSystem::write(
    sim::EntityComponentManager* ecm) {
  if (!(this->command_interface_)) {
    return;
  }
  // Note that we don't have to take care of mimic joints,
  // it's handled in mimic joint settings in parent.
  const double hand_l_torque = this->hand_l_spring_proximal_joint_pos_ * this->hand_spring_coeff_;
  const double hand_r_torque = this->hand_r_spring_proximal_joint_pos_ * this->hand_spring_coeff_;

  this->drive_mode_ = this->drive_mode_cmd_;
  if (this->drive_mode_ == tmc_exxx_servo_motor_protocol::kDriveModeHandPosition) {
    command_interface_->set_value(this->motor_joint_.joint_position_cmd);
  } else if (this->drive_mode_ == tmc_exxx_servo_motor_protocol::kDriveModeHandGrasp) {
    command_interface_->set_value(this->motor_joint_.joint_position);

    // TODO(Takeshita) パラメータ調整
    if (this->grasping_flag_ > 0.0) {
      const double effort_error = this->motor_joint_.joint_effort - this->motor_joint_.joint_effort_cmd;
      const double target_velocity = -(this->grasp_velocity_gain_ * effort_error);

      auto vel = ecm->Component<sim::components::JointVelocityCmd>(this->motor_joint_.sim_joint);
      if (vel == nullptr) {
        ecm->CreateComponent(
            this->motor_joint_.sim_joint,
            sim::components::JointVelocityCmd({target_velocity}));
      } else if (!vel->Data().empty()) {
        vel->Data()[0] = target_velocity;
      }

      if (std::abs(effort_error) < this->grasp_effort_tolerance_) {
        this->grasping_flag_ = -1.0;
      }
    }
    if (this->grasping_flag_cmd_ > 0.0) {
      this->grasping_flag_ = 1.0;
    }
  }

  // gripper
  // Not accurate, provisional implementation, vibrates without d
  constexpr double d_gain = 0.1;

  const double hand_l_torque_cmd = -hand_l_torque - d_gain * this->hand_l_spring_proximal_joint_vel_;
  if (!ecm->Component<sim::components::JointForceCmd>(this->hand_l_spring_proximal_joint_)) {
    ecm->CreateComponent(
        this->hand_l_spring_proximal_joint_,
        sim::components::JointForceCmd({hand_l_torque_cmd}));
  } else {
    const auto cmd = ecm->Component<sim::components::JointForceCmd>(this->hand_l_spring_proximal_joint_);
    *cmd = sim::components::JointForceCmd({hand_l_torque_cmd});
  }

  const double hand_r_torque_cmd = -hand_r_torque - d_gain * this->hand_r_spring_proximal_joint_vel_;
  if (!ecm->Component<sim::components::JointForceCmd>(this->hand_r_spring_proximal_joint_)) {
    ecm->CreateComponent(this->hand_r_spring_proximal_joint_,
        sim::components::JointForceCmd({hand_r_torque_cmd}));
  } else {
    const auto cmd = ecm->Component<sim::components::JointForceCmd>(this->hand_r_spring_proximal_joint_);
    *cmd = sim::components::JointForceCmd({hand_r_torque_cmd});
  }
}


bool GazeboSimGripperSystem::initGripper(
    const std::map<std::string, sim::Entity>& enableJoints,
    const hardware_interface::HardwareInfo& hardware_info,
    const std::string& prefix,
    std::vector<hardware_interface::CommandInterface>& command_interfaces,
    std::vector<hardware_interface::CommandInterface>& parent_command_interfaces,
    std::vector<hardware_interface::StateInterface>& state_interfaces,
    std::vector<hardware_interface::StateInterface>& parent_state_interfaces) {
  this->motor_joint_.name = "hand_" + prefix + "motor_joint";
  this->grasp_velocity_gain_ = kGraspVelocityGain;
  this->grasp_effort_tolerance_ = kGraspEffortTolerance;

  auto set_joint = [&](const std::string& joint_name, sim::Entity& out_entity) -> bool {
    auto it = enableJoints.find(joint_name);
    if (it == enableJoints.end()) {
      return false;
    }
    out_entity = it->second;
    return true;
  };

  if (!set_joint(this->motor_joint_.name, this->motor_joint_.sim_joint)) {
    return false;
  }
  if (!set_joint("hand_" + prefix + "left_spring_proximal_joint", this->hand_l_spring_proximal_joint_) &&
      !set_joint("hand_" + prefix + "l_spring_proximal_joint", this->hand_l_spring_proximal_joint_)) {
    return false;
  }
  if (!set_joint("hand_" + prefix + "right_spring_proximal_joint", this->hand_r_spring_proximal_joint_) &&
      !set_joint("hand_" + prefix + "r_spring_proximal_joint", this->hand_r_spring_proximal_joint_)) {
    return false;
  }

  // Inherit Interface from parent. Remove what was used here
  for (auto it = parent_command_interfaces.begin(); it != parent_command_interfaces.end(); ++it) {
    if (it->get_prefix_name() == this->motor_joint_.name &&
        it->get_interface_name() == hardware_interface::HW_IF_POSITION) {
      this->command_interface_ = std::make_unique<hardware_interface::CommandInterface>(std::move(*it));
      break;
    }
  }
  if (!this->command_interface_) {
    return false;
  }

  for (auto it = parent_state_interfaces.begin(); it != parent_state_interfaces.end(); ++it) {
    if (it->get_prefix_name() == this->motor_joint_.name &&
        it->get_interface_name() == hardware_interface::HW_IF_EFFORT) {
      // Discard this as it won't be used later
      auto _ = std::make_unique<hardware_interface::StateInterface>(std::move(*it));
      break;
    }
    if (it->get_prefix_name() == this->motor_joint_.name &&
        it->get_interface_name() == hardware_interface::HW_IF_POSITION) {
      this->state_interface_ = std::make_unique<hardware_interface::StateInterface>(std::move(*it));
      break;
    }
  }
  if (!this->state_interface_) {
    return false;
  }

  // define command interfaces
  command_interfaces.emplace_back(
      this->motor_joint_.name,
      hardware_interface::HW_IF_POSITION,
      &this->motor_joint_.joint_position_cmd);
  command_interfaces.emplace_back(
      this->motor_joint_.name,
      hardware_interface::HW_IF_EFFORT,
      &this->motor_joint_.joint_effort_cmd);
  command_interfaces.emplace_back(
      this->motor_joint_.name, "command_drive_mode", &this->drive_mode_cmd_);
  command_interfaces.emplace_back(
      this->motor_joint_.name, "command_grasping_flag", &this->grasping_flag_cmd_);

  // define state interfaces
  state_interfaces.emplace_back(
      this->motor_joint_.name,
      hardware_interface::HW_IF_POSITION,
      &this->motor_joint_.joint_position);
  state_interfaces.emplace_back(
      this->motor_joint_.name,
      hardware_interface::HW_IF_EFFORT,
      &this->motor_joint_.joint_effort);
  state_interfaces.emplace_back(
      this->motor_joint_.name,
      "current_drive_mode",
      &this->drive_mode_);
  state_interfaces.emplace_back(
      this->motor_joint_.name,
      "current_grasping_flag",
      &this->grasping_flag_);
  state_interfaces.emplace_back(
      this->motor_joint_.name,
      "current",
      &this->current_);

  auto it = std::find_if(
      hardware_info.joints.begin(), hardware_info.joints.end(),
      [&](const auto& joint) {
        return joint.name == this->motor_joint_.name;
      });

  if (it != hardware_info.joints.end()) {
    auto gain_param = it->parameters.find("grasp_velocity_gain");
    if (gain_param != it->parameters.end()) {
      this->grasp_velocity_gain_ = std::stod(gain_param->second);
    }
    auto tolerance_param = it->parameters.find("grasp_effort_tolerance");
    if (tolerance_param != it->parameters.end()) {
      this->grasp_effort_tolerance_ = std::stod(tolerance_param->second);
    }
  }
  return true;
}

bool GazeboSimSystem::initSim(
    rclcpp::Node::SharedPtr& model_nh,
    std::map<std::string, sim::Entity>& enableJoints,
    const hardware_interface::HardwareInfo& hardware_info,
    sim::EntityComponentManager& _ecm,
    int& update_rate) {
  this->nh_ = model_nh;

  try {
    this->parent_ = std::unique_ptr<gz_ros2_control::GazeboSimSystemInterface>(
        this->gz_system_loader_.createUnmanagedInstance("gz_ros2_control/GazeboSimSystem"));
  } catch (pluginlib::PluginlibException &ex) {
    RCLCPP_ERROR(this->nh_->get_logger(),
        "The plugin failed to load for some reason. Error: %s\n", ex.what());
    return false;
  }

  this->ecm_ = &_ecm;
  this->parent_->initSim(model_nh, enableJoints, hardware_info, _ecm, update_rate);

  auto parent_command_interfaces = this->parent_->export_command_interfaces();
  auto parent_state_interfaces = this->parent_->export_state_interfaces();

  // TODO(MasayukiMasuda): configなど読み込んで必要なものだけ立ち上げるようにする
  const std::vector<std::string> prefixes = {"", "right_", "left_"};
  for (const auto& prefix : prefixes) {
    auto gripper_system = std::make_unique<GazeboSimGripperSystem>();
    if (gripper_system->initGripper(
        enableJoints,
        hardware_info,
        prefix,
        this->command_interfaces_,
        parent_command_interfaces,
        this->state_interfaces_,
        parent_state_interfaces)) {
      this->gripper_systems_.push_back(std::move(gripper_system));
    } else {
      RCLCPP_WARN(this->nh_->get_logger(), "Joint with prefix '%s' not found in enableJoints.", prefix.c_str());
      RCLCPP_WARN(this->nh_->get_logger(), "Will NOT add this empty joint to gripper system.");
    }
  }

  // Skip moved elements and inherit the remaining Interface from parent
  for (auto& command_interface : parent_command_interfaces) {
    if (command_interface.get_prefix_name().empty()) {
      continue;
    }
    this->command_interfaces_.push_back(std::move(command_interface));
  }
  for (auto& state_interface : parent_state_interfaces) {
    if (state_interface.get_prefix_name().empty()) {
      continue;
    }
    this->state_interfaces_.push_back(std::move(state_interface));
  }

  for (const auto& joint : hardware_info.joints) {
    for (const auto& param : joint.parameters) {
      if ((param.first == "do_saturate") &&
          (param.second == "true" || param.second == "True" || param.second == "1")) {
        JointSaturation saturation;
        saturation.joint = enableJoints[joint.name];
        const auto axis = _ecm.Component<sim::components::JointAxis>(saturation.joint)->Data();
        saturation.upper = axis.Upper();
        saturation.lower = axis.Lower();
        this->joint_saturations_.push_back(saturation);
      }
    }
  }
  return true;
}

CallbackReturn GazeboSimSystem::on_init(const hardware_interface::HardwareInfo& info) {
  return this->parent_->on_init(info);
}

CallbackReturn GazeboSimSystem::on_configure(const rclcpp_lifecycle::State& previous_state) {
  return this->parent_->on_configure(previous_state);
}

std::vector<hardware_interface::StateInterface> GazeboSimSystem::export_state_interfaces() {
  return std::move(this->state_interfaces_);
}

std::vector<hardware_interface::CommandInterface> GazeboSimSystem::export_command_interfaces() {
  return std::move(this->command_interfaces_);
}

CallbackReturn GazeboSimSystem::on_activate(const rclcpp_lifecycle::State& previous_state) {
  return this->parent_->on_activate(previous_state);
}

CallbackReturn GazeboSimSystem::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
  return this->parent_->on_deactivate(previous_state);
}

hardware_interface::return_type GazeboSimSystem::read(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) {
  auto ret = this->parent_->read(time, period);
  if (ret != hardware_interface::return_type::OK) {
    return ret;
  }

  for (auto& gripper_system : this->gripper_systems_) {
    gripper_system->read(this->ecm_);
  }
  return ret;
}

hardware_interface::return_type GazeboSimSystem::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  return this->parent_->perform_command_mode_switch(start_interfaces, stop_interfaces);
}

hardware_interface::return_type GazeboSimSystem::write(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) {
  const auto result = this->parent_->write(time, period);

  for (auto& gripper_system : this->gripper_systems_) {
    gripper_system->write(this->ecm_);
  }

  constexpr double kJointResetPosition = 1.0e-4;
  for (const auto& saturation : joint_saturations_) {
    const auto joint_pos =
        this->ecm_->Component<sim::components::JointPosition>(saturation.joint)->Data()[0];

    std::optional<double> reset_position = std::nullopt;
    if (joint_pos < saturation.lower) {
      reset_position = saturation.lower + kJointResetPosition;
    } else if (joint_pos > saturation.upper) {
      reset_position = saturation.upper - kJointResetPosition;
    }

    if (reset_position.has_value()) {
      auto command_position = this->ecm_->Component<sim::components::JointPositionReset>(saturation.joint);
      if (command_position == nullptr) {
        this->ecm_->CreateComponent(
            saturation.joint, sim::components::JointPositionReset({reset_position.value()}));
      } else {
        command_position->Data()[0] = reset_position.value();
      }
    }
  }
  return result;
}
}  // namespace hsrb_gz_ros2_control

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(hsrb_gz_ros2_control::GazeboSimSystem, gz_ros2_control::GazeboSimSystemInterface)

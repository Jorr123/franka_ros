// Copyright (c) 2026
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

#include <franka_example_controllers/isaac_joint_stream_controller2.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <exception>
#include <string>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
IsaacJointStreamController2::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joints_) {
    config.names.push_back(joint + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
IsaacJointStreamController2::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joints_) {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
  }
  return config;
}

controller_interface::return_type IsaacJointStreamController2::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  update_joint_states();

  std::array<double, num_joints> candidate_target;
  rclcpp::Time candidate_time{0, 0, get_node()->get_clock()->get_clock_type()};
  bool has_new_candidate = false;
  {
    std::unique_lock<std::mutex> lock(target_mutex_, std::try_to_lock);
    if (lock.owns_lock() && has_latest_target_ &&
        latest_target_sequence_ != consumed_target_sequence_) {
      candidate_target = latest_target_q_;
      candidate_time = latest_target_time_;
      consumed_target_sequence_ = latest_target_sequence_;
      has_new_candidate = true;
    }
  }

  if (has_new_candidate) {
    const double target_age = (get_node()->now() - candidate_time).seconds();
    if (target_age > target_timeout_) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "Ignoring stale Isaac controller2 target.");
    } else if (motion_active_ && !allow_goal_preemption_ &&
               !target_matches_active_goal(candidate_target)) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "Ignoring new Isaac controller2 target while previous motion is active.");
    } else if (target_within_joint_limits(candidate_target) &&
               target_close_to_current(candidate_target)) {
      if (!motion_active_ || allow_goal_preemption_ ||
          !target_matches_active_goal(candidate_target)) {
        start_motion_to_target(candidate_target);
      }
    }
  }

  if (!motion_active_) {
    set_zero_effort_commands();
    return controller_interface::return_type::OK;
  }

  auto motion_generator_output =
      motion_generator_->getDesiredJointPositions(get_node()->now() - trajectory_start_time_);
  Vector7d q_desired = motion_generator_output.first;
  const bool trajectory_finished = motion_generator_output.second;

  const double kAlpha = 0.99;
  dq_filtered_ = (1.0 - kAlpha) * dq_filtered_ + kAlpha * dq_;

  const double max_error = (q_goal_ - q_).cwiseAbs().maxCoeff();
  if (trajectory_finished && max_error <= goal_tolerance_) {
    motion_active_ = false;
    motion_generator_.reset();
    set_zero_effort_commands();
    RCLCPP_INFO(get_node()->get_logger(), "Isaac controller2 reached goal.");
    return controller_interface::return_type::OK;
  }

  Vector7d tau_d_calculated =
      k_gains_.cwiseProduct(q_desired - q_) + d_gains_.cwiseProduct(-dq_filtered_);
  for (size_t i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d_calculated(static_cast<int>(i)));
  }

  return controller_interface::return_type::OK;
}

CallbackReturn IsaacJointStreamController2::on_init() {
  try {
    auto_declare<std::string>("robot_type", robot_type_);
    auto_declare<std::string>("target_topic", target_topic_);
    auto_declare<std::vector<std::string>>("joints", joints_);
    auto_declare<double>("speed_factor", speed_factor_);
    auto_declare<double>("max_goal_delta", max_goal_delta_);
    auto_declare<double>("target_timeout", target_timeout_);
    auto_declare<double>("goal_tolerance", goal_tolerance_);
    auto_declare<bool>("allow_goal_preemption", allow_goal_preemption_);
    auto_declare<std::vector<double>>("k_gains",
                                      {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0});
    auto_declare<std::vector<double>>("d_gains",
                                      {30.0, 30.0, 30.0, 30.0, 10.0, 10.0, 5.0});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn IsaacJointStreamController2::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  robot_type_ = get_node()->get_parameter("robot_type").as_string();
  target_topic_ = get_node()->get_parameter("target_topic").as_string();
  joints_ = get_node()->get_parameter("joints").as_string_array();
  speed_factor_ = get_node()->get_parameter("speed_factor").as_double();
  max_goal_delta_ = get_node()->get_parameter("max_goal_delta").as_double();
  target_timeout_ = get_node()->get_parameter("target_timeout").as_double();
  goal_tolerance_ = get_node()->get_parameter("goal_tolerance").as_double();
  allow_goal_preemption_ = get_node()->get_parameter("allow_goal_preemption").as_bool();

  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();

  if (joints_.size() != num_joints || k_gains.size() != num_joints ||
      d_gains.size() != num_joints) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "joints, k_gains, and d_gains must each contain exactly %zu values.",
                 num_joints);
    return CallbackReturn::ERROR;
  }
  if (speed_factor_ <= 0.0 || speed_factor_ > 1.0 || max_goal_delta_ <= 0.0 ||
      target_timeout_ <= 0.0 || goal_tolerance_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "speed_factor must be in (0, 1], and max_goal_delta, target_timeout, "
                 "goal_tolerance must be positive.");
    return CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < num_joints; ++i) {
    k_gains_(static_cast<int>(i)) = k_gains.at(i);
    d_gains_(static_cast<int>(i)) = d_gains.at(i);
  }

  target_subscription_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      target_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::array<double, num_joints> parsed_target;
        if (!fill_target_from_msg(*msg, parsed_target)) {
          RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                               "Ignoring Isaac controller2 target with missing names or positions.");
          return;
        }

        std::lock_guard<std::mutex> lock(target_mutex_);
        latest_target_q_ = parsed_target;
        latest_target_time_ = get_node()->now();
        ++latest_target_sequence_;
        has_latest_target_ = true;
      });

  RCLCPP_INFO(get_node()->get_logger(),
              "Isaac controller2 configured: topic=%s, speed_factor=%.4f, "
              "max_goal_delta=%.4f rad, target_timeout=%.3f s.",
              target_topic_.c_str(), speed_factor_, max_goal_delta_, target_timeout_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn IsaacJointStreamController2::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  update_joint_states();
  dq_filtered_.setZero();
  q_goal_ = q_;
  motion_generator_.reset();
  motion_active_ = false;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    has_latest_target_ = false;
    latest_target_sequence_ = 0;
    consumed_target_sequence_ = 0;
  }
  set_zero_effort_commands();
  return CallbackReturn::SUCCESS;
}

CallbackReturn IsaacJointStreamController2::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  motion_active_ = false;
  motion_generator_.reset();
  set_zero_effort_commands();
  return CallbackReturn::SUCCESS;
}

void IsaacJointStreamController2::update_joint_states() {
  for (size_t i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(static_cast<int>(i)) = position_interface.get_value();
    dq_(static_cast<int>(i)) = velocity_interface.get_value();
  }
}

bool IsaacJointStreamController2::fill_target_from_msg(
    const sensor_msgs::msg::JointState& msg,
    std::array<double, num_joints>& target) const {
  if (msg.name.size() != msg.position.size()) {
    return false;
  }

  for (size_t joint_index = 0; joint_index < num_joints; ++joint_index) {
    const auto name_it = std::find(msg.name.begin(), msg.name.end(), joints_[joint_index]);
    if (name_it == msg.name.end()) {
      return false;
    }

    const auto msg_index = static_cast<size_t>(std::distance(msg.name.begin(), name_it));
    const double position = msg.position[msg_index];
    if (!std::isfinite(position)) {
      return false;
    }
    target[joint_index] = position;
  }

  return true;
}

bool IsaacJointStreamController2::target_within_joint_limits(
    const std::array<double, num_joints>& target) const {
  constexpr double limit_margin = 1e-6;
  for (size_t i = 0; i < num_joints; ++i) {
    if (target[i] < lower_joint_limits_[i] - limit_margin ||
        target[i] > upper_joint_limits_[i] + limit_margin) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "Ignoring Isaac controller2 target outside FR3 joint limits.");
      return false;
    }
  }
  return true;
}

bool IsaacJointStreamController2::target_close_to_current(
    const std::array<double, num_joints>& target) const {
  for (size_t i = 0; i < num_joints; ++i) {
    if (std::abs(target[i] - q_(static_cast<int>(i))) > max_goal_delta_) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "Ignoring Isaac controller2 target because it is too far from current "
                           "measured position.");
      return false;
    }
  }
  return true;
}

bool IsaacJointStreamController2::target_reached(
    const std::array<double, num_joints>& target) const {
  for (size_t i = 0; i < num_joints; ++i) {
    if (std::abs(target[i] - q_(static_cast<int>(i))) > goal_tolerance_) {
      return false;
    }
  }
  return true;
}

bool IsaacJointStreamController2::target_matches_active_goal(
    const std::array<double, num_joints>& target) const {
  for (size_t i = 0; i < num_joints; ++i) {
    if (std::abs(target[i] - q_goal_(static_cast<int>(i))) > 1e-6) {
      return false;
    }
  }
  return true;
}

void IsaacJointStreamController2::set_zero_effort_commands() {
  for (auto& command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }
}

void IsaacJointStreamController2::start_motion_to_target(
    const std::array<double, num_joints>& target) {
  for (size_t i = 0; i < num_joints; ++i) {
    q_goal_(static_cast<int>(i)) = target[i];
  }
  if (target_reached(target)) {
    motion_active_ = false;
    motion_generator_.reset();
    set_zero_effort_commands();
    return;
  }
  motion_generator_ = std::make_unique<MotionGenerator>(speed_factor_, q_, q_goal_);
  trajectory_start_time_ = get_node()->now();
  motion_active_ = true;
  RCLCPP_INFO(get_node()->get_logger(), "Isaac controller2 accepted new MotionGenerator goal.");
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::IsaacJointStreamController2,
                       controller_interface::ControllerInterface)

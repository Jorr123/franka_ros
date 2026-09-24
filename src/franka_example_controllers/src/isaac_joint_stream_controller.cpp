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

#include <franka_example_controllers/isaac_joint_stream_controller.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <string>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
IsaacJointStreamController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joints_) {
    config.names.push_back(joint + "/position");
  }
  return config;
}

controller_interface::InterfaceConfiguration
IsaacJointStreamController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joints_) {
    config.names.push_back(joint + "/position");
  }
  return config;
}

controller_interface::return_type IsaacJointStreamController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  std::array<double, num_joints> current_q;
  for (size_t i = 0; i < num_joints; ++i) {
    current_q[i] = state_interfaces_[i].get_value();
  }

  if (!initialized_) {
    desired_q_ = current_q;
    desired_dq_.fill(0.0);
    target_q_ = current_q;
    initialized_ = true;
  }

  std::array<double, num_joints> candidate_target;
  uint64_t candidate_sequence = consumed_target_sequence_;
  bool has_candidate = false;
  target_age_ += period.seconds();
  {
    std::unique_lock<std::mutex> lock(target_mutex_, std::try_to_lock);
    if (lock.owns_lock() && has_latest_target_) {
      candidate_target = latest_target_q_;
      candidate_sequence = latest_target_sequence_;
      has_candidate = true;
    }
  }
  if (candidate_sequence != consumed_target_sequence_) {
    consumed_target_sequence_ = candidate_sequence;
    target_age_ = 0.0;
  }

  if (has_candidate && target_age_ <= target_timeout_) {
    if (target_within_joint_limits(candidate_target) &&
        target_close_to_current(candidate_target, current_q)) {
      target_q_ = candidate_target;
    } else {
      hold_desired_position();
    }
  } else {
    hold_desired_position();
  }

  const double dt = std::clamp(period.seconds(), 0.0, max_filter_dt_);
  const double max_delta_velocity = std::max(0.0, max_acceleration_) * dt;
  for (size_t i = 0; i < num_joints; ++i) {
    const double error = target_q_[i] - desired_q_[i];
    const double abs_error = std::abs(error);

    if (abs_error < 1e-8 && std::abs(desired_dq_[i]) <= max_delta_velocity) {
      desired_q_[i] = target_q_[i];
      desired_dq_[i] = 0.0;
      command_interfaces_[i].set_value(desired_q_[i]);
      continue;
    }

    const double direction = (error > 0.0) - (error < 0.0);
    const double stopping_velocity = std::sqrt(2.0 * std::max(0.0, max_acceleration_) * abs_error);
    const double target_velocity =
        direction * std::min(std::max(0.0, max_velocity_), stopping_velocity);

    const double velocity_delta =
        std::clamp(target_velocity - desired_dq_[i], -max_delta_velocity, max_delta_velocity);
    desired_dq_[i] += velocity_delta;

    const double position_step = desired_dq_[i] * dt;
    if (std::abs(position_step) >= abs_error) {
      desired_q_[i] = target_q_[i];
      desired_dq_[i] = 0.0;
    } else {
      desired_q_[i] += position_step;
    }
    command_interfaces_[i].set_value(desired_q_[i]);
  }

  return controller_interface::return_type::OK;
}

CallbackReturn IsaacJointStreamController::on_init() {
  try {
    auto_declare<std::string>("robot_type", robot_type_);
    auto_declare<std::string>("target_topic", target_topic_);
    auto_declare<double>("max_velocity", max_velocity_);
    auto_declare<double>("max_acceleration", max_acceleration_);
    auto_declare<double>("max_filter_dt", max_filter_dt_);
    auto_declare<double>("max_target_jump", max_target_jump_);
    auto_declare<double>("target_timeout", target_timeout_);
    auto_declare<std::vector<std::string>>("joints", joints_);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn IsaacJointStreamController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  robot_type_ = get_node()->get_parameter("robot_type").as_string();
  target_topic_ = get_node()->get_parameter("target_topic").as_string();
  max_velocity_ = get_node()->get_parameter("max_velocity").as_double();
  max_acceleration_ = get_node()->get_parameter("max_acceleration").as_double();
  max_filter_dt_ = get_node()->get_parameter("max_filter_dt").as_double();
  max_target_jump_ = get_node()->get_parameter("max_target_jump").as_double();
  target_timeout_ = get_node()->get_parameter("target_timeout").as_double();
  joints_ = get_node()->get_parameter("joints").as_string_array();

  if (joints_.size() != num_joints) {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu joints, got %zu.", num_joints,
                 joints_.size());
    return CallbackReturn::ERROR;
  }
  if (max_velocity_ <= 0.0 || max_acceleration_ <= 0.0 || max_filter_dt_ <= 0.0 ||
      target_timeout_ <= 0.0 || max_target_jump_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "max_velocity, max_acceleration, max_filter_dt, target_timeout, and "
                 "max_target_jump must be positive.");
    return CallbackReturn::ERROR;
  }

  target_subscription_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      target_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::array<double, num_joints> parsed_target;
        if (!fill_target_from_msg(*msg, parsed_target)) {
          RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                               "Ignoring Isaac joint target with missing names or positions.");
          return;
        }

        std::lock_guard<std::mutex> lock(target_mutex_);
        latest_target_q_ = parsed_target;
        ++latest_target_sequence_;
        has_latest_target_ = true;
      });

  RCLCPP_INFO(get_node()->get_logger(),
              "Isaac joint stream controller configured: topic=%s, max_velocity=%.4f rad/s, "
              "max_acceleration=%.4f rad/s^2, max_filter_dt=%.4f s, max_target_jump=%.4f rad, "
              "target_timeout=%.3f s.",
              target_topic_.c_str(), max_velocity_, max_acceleration_, max_filter_dt_,
              max_target_jump_, target_timeout_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn IsaacJointStreamController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  initialized_ = false;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    has_latest_target_ = false;
    latest_target_sequence_ = 0;
    consumed_target_sequence_ = 0;
    target_age_ = std::numeric_limits<double>::infinity();
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn IsaacJointStreamController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  hold_desired_position();
  return CallbackReturn::SUCCESS;
}

bool IsaacJointStreamController::fill_target_from_msg(
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

bool IsaacJointStreamController::target_within_joint_limits(
    const std::array<double, num_joints>& target) const {
  constexpr double limit_margin = 1e-6;
  for (size_t i = 0; i < num_joints; ++i) {
    if (target[i] < lower_joint_limits_[i] - limit_margin ||
        target[i] > upper_joint_limits_[i] + limit_margin) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "Ignoring Isaac joint target outside FR3 joint limits.");
      return false;
    }
  }
  return true;
}

bool IsaacJointStreamController::target_close_to_current(
    const std::array<double, num_joints>& target,
    const std::array<double, num_joints>& current) const {
  for (size_t i = 0; i < num_joints; ++i) {
    if (std::abs(target[i] - current[i]) > max_target_jump_) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "Ignoring Isaac joint target because it is too far from current "
                           "measured position.");
      return false;
    }
  }
  return true;
}

void IsaacJointStreamController::hold_desired_position() {
  target_q_ = desired_q_;
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::IsaacJointStreamController,
                       controller_interface::ControllerInterface)

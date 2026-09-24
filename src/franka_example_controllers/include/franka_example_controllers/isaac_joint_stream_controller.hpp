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

#pragma once

#include <array>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

class IsaacJointStreamController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  static constexpr size_t num_joints = 7;

  bool fill_target_from_msg(const sensor_msgs::msg::JointState& msg,
                            std::array<double, num_joints>& target) const;
  bool target_within_joint_limits(const std::array<double, num_joints>& target) const;
  bool target_close_to_current(const std::array<double, num_joints>& target,
                               const std::array<double, num_joints>& current) const;
  void hold_desired_position();

  std::string robot_type_{"fr3"};
  std::string target_topic_{"/isaac_joint_targets"};
  std::vector<std::string> joints_{"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
                                   "fr3_joint5", "fr3_joint6", "fr3_joint7"};

  double max_velocity_{0.02};
  double max_acceleration_{0.002};
  double max_filter_dt_{0.001};
  double max_target_jump_{0.1};
  double target_timeout_{0.5};

  const std::array<double, num_joints> lower_joint_limits_{
      {-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159}};
  const std::array<double, num_joints> upper_joint_limits_{
      {2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159}};

  std::array<double, num_joints> desired_q_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, num_joints> desired_dq_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, num_joints> target_q_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

  std::mutex target_mutex_;
  std::array<double, num_joints> latest_target_q_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  uint64_t latest_target_sequence_{0};
  uint64_t consumed_target_sequence_{0};
  double target_age_{std::numeric_limits<double>::infinity()};
  bool has_latest_target_{false};
  bool initialized_{false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_subscription_;
};

}  // namespace franka_example_controllers

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
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "franka_example_controllers/motion_generator.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

class IsaacJointStreamController2 : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;

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

  void update_joint_states();
  bool fill_target_from_msg(const sensor_msgs::msg::JointState& msg,
                            std::array<double, num_joints>& target) const;
  bool target_within_joint_limits(const std::array<double, num_joints>& target) const;
  bool target_close_to_current(const std::array<double, num_joints>& target) const;
  bool target_reached(const std::array<double, num_joints>& target) const;
  bool target_matches_active_goal(const std::array<double, num_joints>& target) const;
  void set_zero_effort_commands();
  void start_motion_to_target(const std::array<double, num_joints>& target);

  std::string robot_type_{"fr3"};
  std::string target_topic_{"/isaac_joint_targets2"};
  std::vector<std::string> joints_{"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
                                   "fr3_joint5", "fr3_joint6", "fr3_joint7"};

  double speed_factor_{0.02};
  double max_goal_delta_{0.3};
  double target_timeout_{0.5};
  double goal_tolerance_{0.005};
  bool allow_goal_preemption_{false};

  const std::array<double, num_joints> lower_joint_limits_{
      {-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159}};
  const std::array<double, num_joints> upper_joint_limits_{
      {2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159}};

  Vector7d q_;
  Vector7d dq_;
  Vector7d dq_filtered_;
  Vector7d q_goal_;
  Vector7d k_gains_;
  Vector7d d_gains_;

  std::unique_ptr<MotionGenerator> motion_generator_;
  rclcpp::Time trajectory_start_time_;
  bool motion_active_{false};

  std::mutex target_mutex_;
  std::array<double, num_joints> latest_target_q_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  rclcpp::Time latest_target_time_{0, 0, RCL_ROS_TIME};
  uint64_t latest_target_sequence_{0};
  uint64_t consumed_target_sequence_{0};
  bool has_latest_target_{false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_subscription_;
};

}  // namespace franka_example_controllers

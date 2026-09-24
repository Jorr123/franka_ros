# IsaacJointStreamController Code Walkthrough

This document explains the custom controller in beginner-friendly detail. It
assumes you do not already know ROS 2, `ros2_control`, plugin controllers, or
how Isaac Sim commands reach the Franka robot.

The actual controller code lives here:

```text
src/franka_example_controllers/include/franka_example_controllers/isaac_joint_stream_controller.hpp
src/franka_example_controllers/src/isaac_joint_stream_controller.cpp
```

The controller is configured here:

```text
src/franka_bringup/config/controllers.yaml
```

The plugin is registered here:

```text
src/franka_example_controllers/franka_example_controllers.xml
src/franka_example_controllers/CMakeLists.txt
```

## Big Picture

The goal is to move the real Franka Research 3 from joint targets published by
Isaac Sim, while still using the safe official Franka ROS 2 control path:

```text
Isaac Sim
  -> /isaac_joint_targets
  -> IsaacJointStreamController
  -> ros2_control command interfaces
  -> franka_hardware
  -> libfranka
  -> real FR3
```

The controller does not talk directly to `libfranka`. That is intentional.
Direct `libfranka` control would bypass the safety structure already provided
by `ros2_control` and `franka_hardware`.

## ROS 2 Basics Used by This Controller

### Node

A ROS 2 node is a running program or component. Examples in this system:

```text
controller_manager
robot_state_publisher
joint_state_broadcaster
franka_robot_state_broadcaster
isaac_joint_stream_controller
```

The controller itself is loaded inside `controller_manager`, not launched as a
normal standalone executable.

### Topic

A topic is a named communication channel. One program publishes messages, and
another subscribes to them.

This controller subscribes to:

```text
/isaac_joint_targets
```

That means Isaac Sim, or a manual test command, can publish target joint
positions there.

### Message Type

Every ROS topic has a message type. This controller uses:

```text
sensor_msgs/msg/JointState
```

That message contains arrays like:

```yaml
name:
  - fr3_joint1
  - fr3_joint2
position:
  - 0.1
  - -0.2
```

The important part is that names and positions travel together. This is safer
than a raw array, because the controller can check that `fr3_joint1` really maps
to the position intended for `fr3_joint1`.

### Parameters

ROS 2 parameters are runtime settings. This controller has parameters such as:

```yaml
target_topic: "/isaac_joint_targets"
max_velocity: 0.005
max_acceleration: 0.002
max_filter_dt: 0.001
max_target_jump: 0.1
target_timeout: 0.5
joints:
  - fr3_joint1
  - fr3_joint2
  - fr3_joint3
  - fr3_joint4
  - fr3_joint5
  - fr3_joint6
  - fr3_joint7
```

These are configured in:

```text
src/franka_bringup/config/controllers.yaml
```

## ros2_control Basics

`ros2_control` is the layer that connects controllers to hardware.

You can think of it as three parts:

```text
hardware read
  -> controller update
  -> hardware write
```

For the Franka robot:

1. `franka_hardware` reads the real robot state from `libfranka`.
2. `controller_manager` calls each active controller's `update()` function.
3. Controllers write command values into command interfaces.
4. `franka_hardware` writes those command values back to `libfranka`.

This controller writes joint position commands. It does not send torque,
Cartesian pose, Cartesian velocity, or direct `libfranka` commands.

## Command Interfaces and State Interfaces

In `ros2_control`, a controller must declare what it wants to use.

### State Interface

A state interface is something the controller reads from the robot.

This controller reads:

```text
fr3_joint1/position
fr3_joint2/position
fr3_joint3/position
fr3_joint4/position
fr3_joint5/position
fr3_joint6/position
fr3_joint7/position
```

These are the current measured joint positions.

### Command Interface

A command interface is something the controller writes to the robot.

This controller writes:

```text
fr3_joint1/position
fr3_joint2/position
fr3_joint3/position
fr3_joint4/position
fr3_joint5/position
fr3_joint6/position
fr3_joint7/position
```

That means it commands joint positions, not joint velocities or torques.

## Why This Controller Exists

The existing example controllers are useful examples, but not suitable for
streaming Isaac targets directly:

- `joint_position_example_controller` moves through its own hardcoded motion.
- `move_to_start_example_controller` moves to a predefined pose.
- `joint_trajectory_controller` accepted trajectories, but caused Franka
  discontinuity errors during small streamed movements in this setup.

This controller was created to be boring and conservative:

- no hardcoded pose
- no automatic movement at startup
- no direct copy from Isaac to command interfaces
- very slow first-test limits
- name-based target validation
- hold position when commands are stale or invalid

## Files Added or Changed

### New Header

```text
src/franka_example_controllers/include/franka_example_controllers/isaac_joint_stream_controller.hpp
```

This file declares the controller class, member variables, and helper functions.

### New Source File

```text
src/franka_example_controllers/src/isaac_joint_stream_controller.cpp
```

This file implements the controller behavior.

### CMake Build File

```text
src/franka_example_controllers/CMakeLists.txt
```

This was updated so the new `.cpp` file is compiled into the
`franka_example_controllers` shared library. It also adds the `sensor_msgs`
dependency because the controller subscribes to `sensor_msgs/msg/JointState`.

### Plugin XML

```text
src/franka_example_controllers/franka_example_controllers.xml
```

This registers the controller class with pluginlib so `controller_manager` can
load it by name.

### Controller YAML

```text
src/franka_bringup/config/controllers.yaml
```

This gives `controller_manager` an instance named:

```text
isaac_joint_stream_controller
```

with type:

```text
franka_example_controllers/IsaacJointStreamController
```

## Header File Walkthrough

The class declaration begins like this:

```cpp
class IsaacJointStreamController : public controller_interface::ControllerInterface
```

This means `IsaacJointStreamController` is a `ros2_control` controller. It must
implement the functions expected by `ControllerInterface`.

### Public Functions

```cpp
command_interface_configuration()
```

This tells `ros2_control` which command interfaces the controller wants to
claim. In this case, the seven FR3 joint position command interfaces.

```cpp
state_interface_configuration()
```

This tells `ros2_control` which state interfaces the controller wants to read.
In this case, the seven FR3 joint position state interfaces.

```cpp
update()
```

This is the most important function. It runs repeatedly while the controller is
active, usually at the controller manager update rate. In this workspace, the
controller manager update rate is configured as 1000 Hz.

At 1000 Hz, `update()` is called about once every millisecond.

```cpp
on_init()
on_configure()
on_activate()
on_deactivate()
```

These are lifecycle hooks. They are called when the controller is initialized,
configured, activated, and deactivated.

## Important Member Variables

### Number of Joints

```cpp
static constexpr size_t num_joints = 7;
```

The FR3 arm has 7 joints, so the controller expects 7 target positions.

### Robot Type

```cpp
std::string robot_type_{"fr3"};
```

The default robot type is `fr3`.

### Target Topic

```cpp
std::string target_topic_{"/isaac_joint_targets"};
```

This is the topic the controller subscribes to.

### Joint Names

```cpp
std::vector<std::string> joints_{
  "fr3_joint1",
  "fr3_joint2",
  "fr3_joint3",
  "fr3_joint4",
  "fr3_joint5",
  "fr3_joint6",
  "fr3_joint7"
};
```

These names define the expected FR3 joints and the internal command order.

Incoming `JointState` messages may have names in a different order. That is OK.
The controller searches by name and builds its own correctly ordered target
array.

### Motion Limits

```cpp
double max_velocity_{0.02};
double max_acceleration_{0.002};
double max_filter_dt_{0.001};
double max_target_jump_{0.1};
double target_timeout_{0.5};
```

The header has fallback defaults. The active values normally come from
`controllers.yaml`.

The YAML currently sets:

```yaml
max_velocity: 0.005
max_acceleration: 0.002
max_filter_dt: 0.001
max_target_jump: 0.1
target_timeout: 0.5
```

Meanings:

- `max_velocity`: fastest allowed filtered joint speed, in radians per second
- `max_acceleration`: fastest allowed change in filtered joint speed, in radians
  per second squared
- `max_filter_dt`: maximum timestep used by the internal filter, so a delayed
  non-realtime update cannot create a large command step
- `max_target_jump`: largest accepted difference between a target and the
  current measured robot position
- `target_timeout`: how old the latest target can be before the controller stops
  following it

### Joint Limits

```cpp
lower_joint_limits_
upper_joint_limits_
```

These arrays contain conservative FR3 joint limits. Incoming targets outside
these limits are ignored.

### Desired and Target Positions

```cpp
std::array<double, num_joints> desired_q_;
std::array<double, num_joints> desired_dq_;
std::array<double, num_joints> target_q_;
```

These are central to the controller.

`target_q_` is the latest valid target the controller wants to approach.

`desired_q_` is the filtered command that is actually written to the robot.

`desired_dq_` is the internal filtered velocity used to ramp smoothly toward the
target.

The controller never writes `target_q_` directly to the robot. It slowly moves
`desired_q_` toward `target_q_`.

### Latest Target From the Subscriber

```cpp
std::array<double, num_joints> latest_target_q_;
uint64_t latest_target_sequence_;
uint64_t consumed_target_sequence_;
double target_age_;
bool has_latest_target_;
```

The ROS subscription callback receives messages whenever something publishes to
`/isaac_joint_targets`.

The real-time `update()` loop then reads the newest target if it can do so
without blocking.

The sequence counters help the update loop know whether a new target arrived.
The age timer helps reject stale commands.

### Mutex

```cpp
std::mutex target_mutex_;
```

The subscription callback and the update loop may run on different threads.
They both access `latest_target_q_`.

The mutex protects that shared data.

Important detail: the update loop uses `std::try_to_lock`, not a blocking lock.
That means if the subscriber is briefly holding the mutex, the real-time update
loop skips that target for this cycle instead of waiting.

That is intentional. A robot control loop should avoid waiting on a normal ROS
callback thread.

## Source File Walkthrough

### command_interface_configuration()

```cpp
controller_interface::InterfaceConfiguration
IsaacJointStreamController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joints_) {
    config.names.push_back(joint + "/position");
  }
  return config;
}
```

This function builds the list of command interfaces the controller needs.

For each joint name, it appends `"/position"`.

So:

```text
fr3_joint1
```

becomes:

```text
fr3_joint1/position
```

The type `INDIVIDUAL` means the controller is asking for exactly these named
interfaces.

When this controller becomes active, `controller_manager` gives it exclusive
ownership of these command interfaces. Another controller cannot command the
same joint position interfaces at the same time.

### state_interface_configuration()

```cpp
controller_interface::InterfaceConfiguration
IsaacJointStreamController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joints_) {
    config.names.push_back(joint + "/position");
  }
  return config;
}
```

This is similar to command interface configuration, but for reading state.

The controller reads each joint's current measured position.

It needs current measured position for three reasons:

1. To initialize safely on startup.
2. To reject targets that are too far away.
3. To compare the filtered command against the real robot state.

### update()

This is the main control loop.

It runs again and again while the controller is active.

The high-level behavior is:

```text
read current robot joint positions
initialize if this is the first update
read latest target from subscriber
reject stale or unsafe target
ramp desired_q toward target_q
write desired_q to command interfaces
```

#### Step 1: Read Current Robot Position

```cpp
std::array<double, num_joints> current_q;
for (size_t i = 0; i < num_joints; ++i) {
  current_q[i] = state_interfaces_[i].get_value();
}
```

`state_interfaces_` is provided by `ros2_control`.

Each `state_interfaces_[i]` contains the measured position for one joint.

The controller copies those values into `current_q`.

#### Step 2: First-Cycle Initialization

```cpp
if (!initialized_) {
  desired_q_ = current_q;
  desired_dq_.fill(0.0);
  target_q_ = current_q;
  initialized_ = true;
}
```

This is one of the most important safety parts.

On the first update after activation:

- the commanded position equals the current measured robot position
- the internal desired velocity is zero
- the target equals the current measured robot position

This prevents the controller from jumping to a hardcoded pose or an old target.

Without this, the first command might be far from the real robot position, which
can trigger Franka reflex errors.

#### Step 3: Try to Read Latest Subscriber Target

```cpp
std::array<double, num_joints> candidate_target;
uint64_t candidate_sequence = consumed_target_sequence_;
bool has_candidate = false;
target_age_ += period.seconds();
```

`candidate_target` is a local copy of the newest target, if one is available.

`target_age_` increases by the time since the previous update. If the controller
runs at 1000 Hz, each cycle adds roughly `0.001` seconds.

Then:

```cpp
std::unique_lock<std::mutex> lock(target_mutex_, std::try_to_lock);
if (lock.owns_lock() && has_latest_target_) {
  candidate_target = latest_target_q_;
  candidate_sequence = latest_target_sequence_;
  has_candidate = true;
}
```

This tries to lock the target mutex. If it succeeds and a target exists, the
controller copies it.

If it cannot lock immediately, it does not wait. It simply keeps going.

#### Step 4: Detect New Target

```cpp
if (candidate_sequence != consumed_target_sequence_) {
  consumed_target_sequence_ = candidate_sequence;
  target_age_ = 0.0;
}
```

When the subscriber receives a message, it increments `latest_target_sequence_`.

The update loop compares that sequence number with `consumed_target_sequence_`.

If they differ, a new target arrived. The controller marks it consumed and
resets `target_age_` to zero.

#### Step 5: Accept or Reject Target

```cpp
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
```

The controller only accepts a target if:

1. A target exists.
2. The target is not stale.
3. The target is inside joint limits.
4. The target is close enough to the measured robot position.

If any check fails, it calls:

```cpp
hold_desired_position();
```

That means the controller stops chasing the target and holds its current
filtered command.

#### Step 6: Smooth Toward Target

```cpp
const double dt = std::clamp(period.seconds(), 0.0, max_filter_dt_);
const double max_delta_velocity = std::max(0.0, max_acceleration_) * dt;
```

`dt` is the update period in seconds.

At 1000 Hz, `dt` is around `0.001`. The code clamps it to `max_filter_dt_`
so that a delayed control cycle cannot produce a larger-than-normal command
step.

`max_delta_velocity` is how much the internal velocity is allowed to change this
cycle.

For each joint:

```cpp
const double error = target_q_[i] - desired_q_[i];
const double abs_error = std::abs(error);
```

`error` is how far the filtered command is from the target.

Then the controller checks whether it is already basically at the target:

```cpp
if (abs_error < 1e-8 && std::abs(desired_dq_[i]) <= max_delta_velocity) {
  desired_q_[i] = target_q_[i];
  desired_dq_[i] = 0.0;
  command_interfaces_[i].set_value(desired_q_[i]);
  continue;
}
```

If the joint is close enough, it snaps the tiny numerical remainder to the
target and sets internal velocity to zero.

#### Step 7: Compute Safe Direction

```cpp
const double direction = (error > 0.0) - (error < 0.0);
```

This produces:

```text
 1  if target is above current desired position
-1  if target is below current desired position
 0  if already at target
```

#### Step 8: Compute Stopping Velocity

```cpp
const double stopping_velocity =
    std::sqrt(2.0 * std::max(0.0, max_acceleration_) * abs_error);
```

This comes from basic motion math:

```text
v^2 = 2 * a * distance
```

It answers: "Given the remaining distance and max acceleration, what velocity
can I still stop from without overshooting?"

As the controller gets close to the target, `abs_error` becomes smaller, so
`stopping_velocity` becomes smaller. That makes the controller slow down before
reaching the target.

#### Step 9: Choose Target Velocity

```cpp
const double target_velocity =
    direction * std::min(std::max(0.0, max_velocity_), stopping_velocity);
```

The controller wants to move in the right direction, but no faster than:

- `max_velocity_`
- the velocity it can safely stop from

This creates a gentle velocity profile:

```text
speed up slowly
cruise if far enough
slow down near target
stop at target
```

#### Step 10: Limit Acceleration

```cpp
const double velocity_delta =
    std::clamp(target_velocity - desired_dq_[i],
               -max_delta_velocity,
               max_delta_velocity);
desired_dq_[i] += velocity_delta;
```

This prevents velocity from changing instantly.

For Franka, this matters a lot. The previous simpler version limited position
step per cycle, but it still allowed velocity to jump from zero to max velocity
immediately. Franka rejected that with:

```text
joint_motion_generator_velocity_discontinuity
joint_motion_generator_acceleration_discontinuity
```

This acceleration limit is the fix for that behavior.

#### Step 11: Integrate Velocity Into Position

```cpp
const double position_step = desired_dq_[i] * dt;
```

Velocity times time gives position change.

Then:

```cpp
if (std::abs(position_step) >= abs_error) {
  desired_q_[i] = target_q_[i];
  desired_dq_[i] = 0.0;
} else {
  desired_q_[i] += position_step;
}
```

If the next step would overshoot the target, the controller lands exactly on the
target and sets velocity to zero.

Otherwise, it takes the small filtered step.

#### Step 12: Write Command Interface

```cpp
command_interfaces_[i].set_value(desired_q_[i]);
```

This is where the controller gives the filtered joint position command to
`ros2_control`.

Important: this does not directly move the robot by itself. It writes into the
command interface. Later in the control cycle, `franka_hardware` writes the
command to `libfranka`.

### on_init()

```cpp
CallbackReturn IsaacJointStreamController::on_init()
```

This function declares parameters.

Examples:

```cpp
auto_declare<std::string>("target_topic", target_topic_);
auto_declare<double>("max_velocity", max_velocity_);
auto_declare<double>("max_acceleration", max_acceleration_);
auto_declare<std::vector<std::string>>("joints", joints_);
```

Declaring parameters tells ROS 2: "This controller accepts these settings."

If declaration throws an exception, the controller returns `ERROR`.

### on_configure()

```cpp
CallbackReturn IsaacJointStreamController::on_configure(...)
```

This runs when you configure the controller:

```bash
ros2 control set_controller_state isaac_joint_stream_controller inactive
```

It reads parameter values:

```cpp
target_topic_ = get_node()->get_parameter("target_topic").as_string();
max_velocity_ = get_node()->get_parameter("max_velocity").as_double();
max_acceleration_ = get_node()->get_parameter("max_acceleration").as_double();
joints_ = get_node()->get_parameter("joints").as_string_array();
```

Then it validates them:

```cpp
if (joints_.size() != num_joints) {
  return CallbackReturn::ERROR;
}
```

The controller refuses to configure if it does not have exactly 7 joints.

It also refuses non-positive safety values:

```cpp
if (max_velocity_ <= 0.0 || max_acceleration_ <= 0.0 ||
    target_timeout_ <= 0.0 || max_target_jump_ <= 0.0) {
  return CallbackReturn::ERROR;
}
```

### Creating the Subscription

Still inside `on_configure()`:

```cpp
target_subscription_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    target_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      ...
    });
```

This tells ROS:

"Whenever a `JointState` message arrives on `/isaac_joint_targets`, call this
lambda function."

A lambda is a small inline function. Here it is the callback for target messages.

Inside the callback:

```cpp
std::array<double, num_joints> parsed_target;
if (!fill_target_from_msg(*msg, parsed_target)) {
  RCLCPP_WARN_THROTTLE(...);
  return;
}
```

The callback first tries to parse and validate the message. If parsing fails,
it logs a throttled warning and ignores the message.

If parsing succeeds:

```cpp
std::lock_guard<std::mutex> lock(target_mutex_);
latest_target_q_ = parsed_target;
++latest_target_sequence_;
has_latest_target_ = true;
```

It safely stores the latest target and increments the sequence number.

### on_activate()

```cpp
CallbackReturn IsaacJointStreamController::on_activate(...)
```

This runs when you activate the controller:

```bash
ros2 control set_controller_state isaac_joint_stream_controller active
```

It resets startup state:

```cpp
initialized_ = false;
```

That ensures the next `update()` cycle initializes from the current measured
robot position.

It also clears old target state:

```cpp
has_latest_target_ = false;
latest_target_sequence_ = 0;
consumed_target_sequence_ = 0;
target_age_ = std::numeric_limits<double>::infinity();
```

This prevents the controller from using a target that was published before
activation.

### on_deactivate()

```cpp
CallbackReturn IsaacJointStreamController::on_deactivate(...)
```

This calls:

```cpp
hold_desired_position();
```

That sets:

```cpp
target_q_ = desired_q_;
```

So the controller's target becomes wherever the filtered command currently is.

## Helper Function: fill_target_from_msg()

This function converts an incoming `JointState` message into the controller's
internal 7-element target array.

The first check:

```cpp
if (msg.name.size() != msg.position.size()) {
  return false;
}
```

A valid `JointState` target must have the same number of names and positions.

Then for each expected joint:

```cpp
const auto name_it = std::find(msg.name.begin(), msg.name.end(), joints_[joint_index]);
```

The controller searches the incoming message for that joint name.

If a joint is missing:

```cpp
return false;
```

If found, it gets the matching position:

```cpp
const auto msg_index = static_cast<size_t>(std::distance(msg.name.begin(), name_it));
const double position = msg.position[msg_index];
```

Then it checks:

```cpp
if (!std::isfinite(position)) {
  return false;
}
```

This rejects `NaN` and infinity values.

Finally:

```cpp
target[joint_index] = position;
```

This stores the target in the controller's expected order.

This is why the incoming message order does not have to match the controller
order. The mapping is by name.

## Helper Function: target_within_joint_limits()

This checks every target joint against hardcoded FR3 limits:

```cpp
if (target[i] < lower_joint_limits_[i] - limit_margin ||
    target[i] > upper_joint_limits_[i] + limit_margin) {
  return false;
}
```

If any target is outside the allowed range, the whole target is rejected.

The controller does not clamp out-of-range targets. It rejects them.

Rejecting is safer than silently modifying a bad command, because a modified
command may hide a bigger upstream problem.

## Helper Function: target_close_to_current()

This checks whether the target is close enough to the real measured robot
position:

```cpp
if (std::abs(target[i] - current[i]) > max_target_jump_) {
  return false;
}
```

With the current YAML:

```yaml
max_target_jump: 0.1
```

Any joint target more than `0.1 rad` away from the current measured position is
rejected.

This prevents a discontinuous Isaac command from asking the real robot to chase
a far-away pose.

## Helper Function: hold_desired_position()

```cpp
void IsaacJointStreamController::hold_desired_position() {
  target_q_ = desired_q_;
}
```

This tells the controller:

"Stop moving toward any old target. Make the current filtered command the new
target."

This is used when:

- no target exists
- the target is stale
- the target is outside joint limits
- the target is too far from the measured robot position

## Plugin Registration

At the bottom of the `.cpp` file:

```cpp
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::IsaacJointStreamController,
                       controller_interface::ControllerInterface)
```

This exports the class as a plugin.

`controller_manager` does not directly know about C++ classes in your source
tree. Pluginlib is how ROS loads controller classes from shared libraries at
runtime.

## XML Plugin Entry

The XML file contains:

```xml
<class name="franka_example_controllers/IsaacJointStreamController"
       type="franka_example_controllers::IsaacJointStreamController"
       base_class_type="controller_interface::ControllerInterface">
```

The `name` is what `controllers.yaml` uses.

The `type` is the actual C++ class name.

The `base_class_type` tells pluginlib this is a `ros2_control` controller.

## CMake Changes

The source file is added to the package library:

```cmake
src/isaac_joint_stream_controller.cpp
```

The package also finds and links `sensor_msgs`:

```cmake
find_package(sensor_msgs REQUIRED)
```

and includes it in:

```cmake
ament_target_dependencies(...)
ament_export_dependencies(...)
```

Without this, the controller would not compile because it uses:

```cpp
#include <sensor_msgs/msg/joint_state.hpp>
```

## YAML Configuration

The controller manager entry:

```yaml
isaac_joint_stream_controller:
  type: franka_example_controllers/IsaacJointStreamController
```

This tells `controller_manager`:

"There is a controller instance named `isaac_joint_stream_controller`, and its
plugin type is `franka_example_controllers/IsaacJointStreamController`."

The parameter block:

```yaml
isaac_joint_stream_controller:
  ros__parameters:
    robot_type: "fr3"
    target_topic: "/isaac_joint_targets"
    max_velocity: 0.005
    max_acceleration: 0.002
    max_filter_dt: 0.001
    max_target_jump: 0.1
    target_timeout: 0.5
    joints:
      - fr3_joint1
      - fr3_joint2
      - fr3_joint3
      - fr3_joint4
      - fr3_joint5
      - fr3_joint6
      - fr3_joint7
```

These values are read by `on_configure()`.

## What Happens When You Run the Controller Commands

### load_controller

```bash
ros2 control load_controller isaac_joint_stream_controller
```

This tells controller manager to load the plugin. ROS uses the YAML type,
plugin XML, and compiled shared library to construct the C++ controller object.

### set inactive

```bash
ros2 control set_controller_state isaac_joint_stream_controller inactive
```

This configures the controller. In this step:

- parameters are read
- the `/isaac_joint_targets` subscription is created
- safety parameters are checked

### set active

```bash
ros2 control set_controller_state isaac_joint_stream_controller active
```

This activates the controller. In this step:

- command interfaces are claimed
- Franka joint position control mode starts through `franka_hardware`
- the controller resets internal state
- the first update initializes from current measured robot position

## What Happens When Isaac Publishes a Target

Suppose Isaac publishes:

```yaml
name:
  - fr3_joint1
  - fr3_joint2
  - fr3_joint3
  - fr3_joint4
  - fr3_joint5
  - fr3_joint6
  - fr3_joint7
position:
  - -0.38
  - -0.25
  - -0.38
  - -1.99
  - 0.00
  - 1.64
  - 0.95
```

The subscriber callback:

1. Checks names and positions have matching lengths.
2. Checks all seven expected FR3 joint names exist.
3. Checks all positions are finite numbers.
4. Stores the target in the controller's internal order.
5. Increments the target sequence number.

Then the next `update()` cycles:

1. Copy the latest target if the mutex is available.
2. Check the target is fresh.
3. Check the target is inside joint limits.
4. Check the target is close to current measured position.
5. Slowly ramp `desired_q_` toward `target_q_`.
6. Write `desired_q_` to the robot command interfaces.

## Why We Use JointState Instead of Float64MultiArray

A raw array like this is risky:

```text
[q1, q2, q3, q4, q5, q6, q7]
```

because both sides must perfectly agree about the order.

`JointState` includes names:

```yaml
name: [fr3_joint1, fr3_joint2, ...]
position: [q1, q2, ...]
```

So the controller can safely map each position to the correct joint.

This matters because a wrong joint order could command a completely different
robot pose.

## Why the Controller Holds Position When Commands Stop

The controller has:

```yaml
target_timeout: 0.5
```

If Isaac stops publishing for more than 0.5 seconds, the controller stops
following the old target.

This prevents the robot from continuing to chase stale data if:

- Isaac freezes
- the network drops
- the publisher crashes
- the bridge stops

## Why Very Small Motion May Be Hard to See

The first-test limits are intentionally tiny:

```yaml
max_velocity: 0.005
max_acceleration: 0.002
```

`0.001 rad` is only about `0.057 degrees`.

`0.01 rad` is only about `0.57 degrees`.

So first tests may sound like the robot is moving but look almost still. Confirm
motion numerically with:

```bash
ros2 topic echo /franka/joint_states --once
```

## Important Limitations

This controller is a conservative first version. It is safer than directly
copying Isaac targets, but it is not a complete production teleoperation safety
system.

Current limitations:

- It uses hardcoded FR3 joint limits.
- It reads only joint position state, not velocity state.
- It does not perform collision checking.
- It does not know the robot workspace or obstacles.
- It does not validate Cartesian end-effector motion.
- It does not check self-collision.
- It does not synchronize clocks with Isaac.
- It assumes targets are joint-space commands for the same FR3 model.

For cautious testing, this is acceptable. For real teleoperation or shared
workspace operation, more safety layers are needed.

## Mental Model

The simplest way to think about the controller:

```text
Incoming Isaac target:
  "I want the robot joints to be here."

Controller:
  "Are all names present?"
  "Are values finite?"
  "Is the command fresh?"
  "Is the target inside joint limits?"
  "Is the target close to the current real robot?"
  "If yes, I will move toward it slowly."
  "If no, I will hold position."

Franka hardware:
  "I will send this filtered joint position command through libfranka."
```

The controller is not trying to be clever. It is trying to be predictable.

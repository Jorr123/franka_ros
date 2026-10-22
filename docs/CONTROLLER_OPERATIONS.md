> Preserved controller operations guide from the original workspace README.
> For a fresh clone, complete [DEVELOPMENT.md](DEVELOPMENT.md) first.
> Commands and addresses below describe the original lab setup.

# Isaac Joint Stream Controller for Franka Research 3

This workspace contains a custom ROS 2 control controller for sending safe joint
position targets from Isaac Sim, or any ROS 2 publisher, to a real Franka
Research 3 robot through `ros2_control`.

The controller is:

```text
franka_example_controllers/IsaacJointStreamController
```

The controller instance name used in `controllers.yaml` is:

```text
isaac_joint_stream_controller
```

It subscribes to:

```text
/isaac_joint_targets
```

with message type:

```text
sensor_msgs/msg/JointState
```

## Safety Notes

Use this only with the robot in a clear, safe workspace and with access to the
robot stop. Start with very small joint movements.

The controller does not directly copy incoming Isaac targets to the robot. It:

- requires all 7 FR3 joint names
- remaps targets by joint name
- rejects missing, incomplete, non-finite, stale, out-of-limit, or large-jump commands
- initializes command position from the current measured robot position
- ramps desired position with velocity and acceleration limits
- holds position when no fresh valid command is available

For a beginner-friendly explanation of the controller code itself, see:

```text
docs/isaac_joint_stream_controller_explained.md
```

The current conservative defaults are:

```yaml
max_velocity: 0.005       # rad/s
max_acceleration: 0.002   # rad/s^2
max_filter_dt: 0.001      # seconds
max_target_jump: 0.1      # rad
target_timeout: 0.5       # seconds
```

## Recommended Clean ROS Domain

If other ROS 2 or DDS participants are on the same network, use a non-default
ROS domain in every terminal:

```bash
export ROS_DOMAIN_ID=99
```

Use the same `ROS_DOMAIN_ID` later on the Windows/Isaac side.

## Build

After changing controller code or YAML:

```bash
cd ~/franka_ros2_ws
colcon build --packages-select franka_example_controllers franka_bringup
source install/setup.bash
```

If only opening a new terminal, sourcing is enough:

```bash
cd ~/franka_ros2_ws
source install/setup.bash
```

## Launch Franka Bringup

In terminal 1:

```bash
cd ~/franka_ros2_ws
export ROS_DOMAIN_ID=99
source install/setup.bash

ros2 launch franka_bringup franka.launch.py robot_ip:=172.16.0.2 robot_type:=fr3
```

Wait until `joint_state_broadcaster` and `franka_robot_state_broadcaster` are
active.

## Load and Activate the Controller

In terminal 2:

```bash
cd ~/franka_ros2_ws
export ROS_DOMAIN_ID=99
source install/setup.bash

ros2 control load_controller isaac_joint_stream_controller
ros2 control set_controller_state isaac_joint_stream_controller inactive
ros2 control set_controller_state isaac_joint_stream_controller active
```

Check controller state:

```bash
ros2 control list_controllers
```

Expected:

```text
isaac_joint_stream_controller  franka_example_controllers/IsaacJointStreamController  active
```

The launch log should show:

```text
Isaac joint stream controller configured: topic=/isaac_joint_targets,
max_velocity=0.0050 rad/s, max_acceleration=0.0020 rad/s^2,
max_target_jump=0.1000 rad, target_timeout=0.500 s.
```

## Verify Real Robot Joint States

Use the Franka-specific topic:

```bash
ros2 topic echo /franka/joint_states --once
```

Expected joint names:

```text
fr3_joint1
fr3_joint2
fr3_joint3
fr3_joint4
fr3_joint5
fr3_joint6
fr3_joint7
```

Do not trust `/joint_states` if another ROS/DDS participant is publishing fake
or simulated joints such as `joint_1` through `joint_6`.

## Send a Small Test Target

First read the current measured state:

```bash
ros2 topic echo /franka/joint_states --once
```

Build a target using the current measured joint positions. Change only one joint
by a very small amount for the first test, such as:

```text
fr3_joint7 += 0.001 rad
```

Publish the target at 20 Hz:

```bash
ros2 topic pub --rate 20 /isaac_joint_targets sensor_msgs/msg/JointState "{
  name: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7],
  position: [-0.386265993, -0.259256363, -0.381168336, -1.993361831, 0.001676804, 1.643216133, 0.946456411]
}"
```

Replace the numbers with the current `/franka/joint_states` values before each
test. The example above is only a sample format.

Stop publishing with `Ctrl+C`.

Because the velocity is very low, tiny movements may be hard to see. Confirm
movement numerically:

```bash
ros2 topic echo /franka/joint_states --once
```

## Send a Larger Joint-Space Move

For larger tests, use the helper script so the final goal is broken into small
intermediate targets:

```bash
python3 scripts/stream_isaac_joint_goal.py --delta fr3_joint7:=0.05
```

The first run is a dry run. To execute:

```bash
python3 scripts/stream_isaac_joint_goal.py --delta fr3_joint7:=0.05 --execute
```

The script reads the current `/joint_states`, computes the requested final goal,
and publishes small waypoints to `/isaac_joint_targets`. Keep total test moves
modest at first, such as `0.05 rad` to `0.10 rad`.

There is also a MotionGenerator-style streamer that mimics the trajectory logic
used by Franka's `move_to_start_example_controller`, without changing
`isaac_joint_stream_controller.cpp`:

```bash
python3 scripts/stream_motion_generator_joint_goal.py --delta fr3_joint7:=0.05
python3 scripts/stream_motion_generator_joint_goal.py --delta fr3_joint7:=0.05 --execute
```

This script still publishes to `/isaac_joint_targets`; the controller remains
the final safety filter.

## Alternative Controller: MotionGenerator + Effort Control

If the position-stream controller still causes Franka motion-generator
discontinuity reflexes, this workspace also contains a second controller:

```text
franka_example_controllers/IsaacJointStreamController2
```

The controller instance name is:

```text
isaac_joint_stream_controller2
```

This controller is closer to Franka's `move_to_start_example_controller`: it
claims the seven joint `effort` command interfaces, uses Franka's
`MotionGenerator` to create a smooth joint-space path, and follows that path
with the same style of joint impedance/PD torque law used in the example.

Do not activate `isaac_joint_stream_controller` and
`isaac_joint_stream_controller2` at the same time. Use only one robot command
controller at a time.

Load and activate controller2:

```bash
ros2 control load_controller isaac_joint_stream_controller2
ros2 control set_controller_state isaac_joint_stream_controller2 inactive
ros2 control set_controller_state isaac_joint_stream_controller2 active
```

Send a single named joint goal to the separate controller2 topic:

```bash
ros2 topic pub --once /isaac_joint_targets2 sensor_msgs/msg/JointState "{
  name: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7],
  position: [-0.474548131, -0.381671280, -0.418461531, -1.836229205, 0.001674044, 1.461098909, 0.979673672]
}"
```

Replace the numbers with your current measured `/joint_states` values and
change only one joint by a small amount first, such as `fr3_joint7 += 0.05`.
The default `max_goal_delta` is `0.3 rad`, so controller2 rejects any goal where
one joint is more than `0.3 rad` away from the current measured robot position.

Controller2 parameters are in `controllers.yaml`:

```yaml
target_topic: "/isaac_joint_targets2"
speed_factor: 0.02
max_goal_delta: 0.3
target_timeout: 0.5
goal_tolerance: 0.005
allow_goal_preemption: false
```

## MoveIt Command-Line Test Scripts

Once MoveIt works from RViz, these scripts let code command MoveIt without
clicking in RViz. They ask MoveIt for a trajectory through
`/plan_kinematic_path`, then only when `--execute` is provided they send the
returned trajectory to `/execute_trajectory`. They do not publish directly to
`ros2_control`.

Start MoveIt first. Do not run `franka_bringup franka.launch.py` at the same
time, because `moveit.launch.py` starts its own `ros2_control_node`.

```bash
cd ~/franka_ros2_ws
export ROS_DOMAIN_ID=99
source install/setup.bash

ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2
```

In another terminal, source the workspace:

```bash
cd ~/franka_ros2_ws
export ROS_DOMAIN_ID=99
source install/setup.bash
```

First test a tiny joint-space plan. This does not move the robot:

```bash
python3 scripts/moveit_joint_goal_test.py --delta fr3_joint7:=0.03
```

If planning succeeds and the RViz preview looks safe, execute the same move:

```bash
python3 scripts/moveit_joint_goal_test.py --delta fr3_joint7:=0.03 --execute
```

For small end-effector offsets, use the guarded Cartesian script. Positive `z`
means positive Z in MoveIt's `base` planning frame. It requests a straight
Cartesian path, preserves end-effector orientation, validates every returned
joint waypoint, and defaults to plan-only. Start with 5 mm:

```bash
python3 scripts/moveit_cartesian_delta_test.py --dz 0.005
```

Then execute only after checking the plan:

```bash
python3 scripts/moveit_cartesian_delta_test.py --dz 0.005 --execute
```

Useful examples:

```bash
python3 scripts/moveit_cartesian_delta_test.py --dz 0.005     # +Z 5 mm, plan only
python3 scripts/moveit_cartesian_delta_test.py --dz -0.005    # -Z 5 mm, plan only
python3 scripts/moveit_cartesian_delta_test.py --dx 0.005     # +X 5 mm, plan only
python3 scripts/moveit_cartesian_delta_test.py --dy 0.005     # +Y 5 mm, plan only
```

Both scripts default to low speed. The joint-goal script uses `0.05`; the
guarded Cartesian script is more conservative at:

```text
maximum planned joint velocity: 0.020 rad/s
maximum planned joint acceleration: 0.050 rad/s^2
```

For Cartesian plans, only execute if the script reports:

```text
Cartesian path fraction: 1.000000
Planning result: SUCCESS
Cartesian trajectory safety check: PASS
```

After execution it also requires fresh measured-state verification and reports
`Final result: VERIFIED CARTESIAN SUCCESS`. The legacy
`moveit_pose_delta_test.py` filename now forwards to this guarded Cartesian
tool. Full instructions and all rejection conditions are in
[docs/relative_cartesian_control.md](relative_cartesian_control.md).

## VLA Joint Command Test

For early VLA/VLM testing, use a safety bridge instead of letting inference
control the robot directly:

```text
VLA output
  -> scripts/vla_joint_command_test.py
  -> validation and small joint deltas
  -> MoveIt planning
  -> optional execution
```

The script accepts mock primitives or JSON. It rejects unknown joints, large
deltas, low confidence, too many moving joints, and targets near joint limits.

Mock primitive, plan only:

```bash
python3 scripts/vla_joint_command_test.py --mock-action rotate_wrist_right
```

Execute only after the plan looks safe:

```bash
python3 scripts/vla_joint_command_test.py --mock-action rotate_wrist_right --execute
```

Example JSON VLA output:

```bash
python3 scripts/vla_joint_command_test.py --json '{
  "action": "joint_delta",
  "joint": "fr3_joint7",
  "delta": 0.05,
  "confidence": 0.9
}'
```

Then execute the same JSON only after planning succeeds:

```bash
python3 scripts/vla_joint_command_test.py --json '{
  "action": "joint_delta",
  "joint": "fr3_joint7",
  "delta": 0.05,
  "confidence": 0.9
}' --execute
```

Expected successful plan output includes:

```text
VLA action accepted: ...
Planning result: SUCCESS
Plan-only run complete. Add --execute to move the robot.
```

### Automatic post-execution verification

Every joint-space script now verifies the real measured state automatically
after a successful `--execute` run. It waits for the robot to settle, requires a
new complete `/joint_states` sample, and checks:

- each commanded joint moved in the requested direction;
- each commanded joint moved by a measurable amount;
- each commanded joint finished within the verification tolerance;
- non-commanded joints did not drift too far; and
- all seven joints are below the settled-velocity limit.

The conservative defaults are:

```text
settle time:                       1.0 s
fresh-state timeout:              2.0 s
target error limit:               0.005 rad
non-commanded joint drift limit:  0.010 rad
maximum settled velocity:         0.010 rad/s
minimum measured movement:        0.001 rad
```

For the tested frozen Qwen action, plan first as usual. Then execute it:

```bash
python3 scripts/vla_joint_command_test.py \
  --json-file /tmp/right_action.json \
  --min-confidence 0.70 \
  --max-delta 0.03 \
  --max-total-delta 0.03 \
  --max-moving-joints 1 \
  --goal-tolerance 0.002 \
  --velocity-scaling 0.03 \
  --acceleration-scaling 0.03 \
  --execute
```

A fully successful run ends with:

```text
Final result: VERIFIED SUCCESS
```

A verification failure returns a nonzero process status and ends with:

```text
Final result: VERIFICATION FAILED - DO NOT SEND ANOTHER COMMAND
```

Treat that as a stopped test: do not send another VLM/VLA action until the
measured state and MoveIt/controller logs have been inspected. The verifier
reports a failure but never attempts an automatic correction. Thresholds can be
changed with `--verification-tolerance`, `--non-commanded-drift-limit`,
`--max-settled-velocity`, `--minimum-measured-delta`, `--settle-time`, and
`--verification-timeout`.

## Qwen VLM Joint Action Test

`scripts/qwen_vlm_joint_action_test.py` asks Qwen-VL to output one JSON action,
then sends that JSON through the same validator used by
`vla_joint_command_test.py`.

The safest first mode uses primitive labels:

```text
Qwen image/text inference
  -> {"action": "primitive", "name": "...", "confidence": ...}
  -> validator converts the primitive to a tiny joint delta
  -> MoveIt plan
  -> optional execution
```

Later, you can test raw joint deltas:

```text
Qwen image/text inference
  -> {"action": "joint_deltas", "deltas": {"fr3_joint7": 0.03}, "confidence": ...}
  -> validator checks size, joint names, joint limits, confidence, and moving joint count
  -> MoveIt plan
  -> optional execution
```

The raw joint-delta mode is closer to what a VLA may output later, but it still
does not bypass the safety bridge.

Install Qwen dependencies only if you want to run the model locally with
Hugging Face Transformers:

```bash
python3 -m pip install -U accelerate qwen-vl-utils pillow torch
python3 -m pip install -U git+https://github.com/huggingface/transformers
```

First test Qwen only. This does not contact ROS or MoveIt:

```bash
python3 scripts/qwen_vlm_joint_action_test.py \
  --image /path/to/camera_image.jpg \
  --task "rotate the wrist slightly to inspect the object" \
  --infer-only
```

If you are running Qwen from a vLLM or SGLang OpenAI-compatible server, use:

```bash
python3 scripts/qwen_vlm_joint_action_test.py \
  --backend openai-compatible \
  --api-url http://localhost:8000/v1/chat/completions \
  --model-id Qwen/Qwen2.5-VL-7B-Instruct \
  --image /path/to/camera_image.jpg \
  --task "rotate the wrist slightly to inspect the object" \
  --infer-only
```

Then start MoveIt:

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2
```

Plan only:

```bash
python3 scripts/qwen_vlm_joint_action_test.py \
  --image /path/to/camera_image.jpg \
  --task "rotate the wrist slightly to inspect the object"
```

Execute only after the plan succeeds and looks safe:

```bash
python3 scripts/qwen_vlm_joint_action_test.py \
  --image /path/to/camera_image.jpg \
  --task "rotate the wrist slightly to inspect the object" \
  --execute
```

To test Qwen raw joint-delta output instead of primitive labels:

```bash
python3 scripts/qwen_vlm_joint_action_test.py \
  --image /path/to/camera_image.jpg \
  --task "make a tiny wrist rotation" \
  --action-space joint-deltas \
  --infer-only
```

Keep `--action-space primitives` for the first real-robot tests. Use
`--action-space joint-deltas` only after the inference-only outputs look stable
and boring.

## Windows-Originated Isaac/Qwen Plan Bridge

The first final-direction bridge keeps static images (and later Isaac frames)
on Windows. A Windows coordinator calls Qwen locally, then sends only an
authenticated, timestamped joint-delta proposal to Ubuntu. Ubuntu validates the
proposal and asks MoveIt to plan.

The supervisor defaults to plan-only and creates no ExecuteTrajectory client in
that mode. See the complete plan-only setup, shared-secret configuration, and
right/left/no-marker tests in
[docs/windows_isaac_plan_bridge.md](windows_isaac_plan_bridge.md).

After that checkpoint passes, the optional supervised mode stores one expiring
frozen plan. It still has no HTTP execution endpoint: only a private local
Ubuntu socket can approve one execution, followed by measured joint-state
verification. Follow
[docs/static_image_supervised_execution.md](static_image_supervised_execution.md).

The next checkpoint runs an ordered collection of Windows images through a
bounded, logged state machine. It requests one MoveIt plan at a time and
refuses to run unless Ubuntu is strictly plan-only. Follow
[docs/static_vla_shadow_session.md](static_vla_shadow_session.md).

After that shadow checkpoint passes, a separate bounded-session supervisor can
execute a very short, locally armed static-image sequence. It permits one joint,
one action in flight, strict sequence numbers, no retries, and requires measured
post-execution verification before the next action. The operator types `ARM`
once; Windows discovers the fresh session automatically and does not require
per-action approval or UUID copying. Follow
[docs/bounded_static_execution_session.md](bounded_static_execution_session.md).

Without a physical camera, the next checkpoint closes the visual loop inside a
synthetic red-marker environment. Each validated Qwen action generates the next
image, and success requires a centered no-action decision. It runs locally or
can require one strict Ubuntu `PLAN_ONLY` result before each mock state update;
it has no robot-execution option. Follow
[docs/mock_visual_closed_loop.md](mock_visual_closed_loop.md).

After both plan-only directions pass, a separate locally armed hardware loop
can execute at most two initial FR3 steps and update the synthetic marker only
from verified measured joint motion. Follow
[docs/synthetic_hardware_loop.md](synthetic_hardware_loop.md).

## Isaac Sim Publishing Format

Isaac Sim should publish a `sensor_msgs/msg/JointState` to:

```text
/isaac_joint_targets
```

The message must include all 7 names and positions:

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
  - q1
  - q2
  - q3
  - q4
  - q5
  - q6
  - q7
```

Isaac should not publish into the real robot root topics:

```text
/joint_states
/franka/joint_states
/robot_description
/tf
/tf_static
```

If Isaac needs its own simulation state, namespace it, for example:

```text
/isaac/joint_states
/isaac/tf
/isaac/robot_description
```

## Error Recovery

If Franka enters reflex or user-stop state, stop publishers and run:

```bash
ros2 action send_goal /action_server/error_recovery franka_msgs/action/ErrorRecovery "{}"
```

Then restart bringup if `ros2_control_node` died.

## Troubleshooting

### `sequence size exceeds remaining buffer`

This usually means another ROS 2/DDS participant is publishing incompatible data
on a topic with the same name. Check:

```bash
ros2 node list --no-daemon
ros2 topic info --verbose /joint_states --no-daemon
ros2 topic info --verbose /robot_description --no-daemon
```

If unknown nodes or unexpected publishers appear, isolate with:

```bash
export ROS_DOMAIN_ID=99
ros2 daemon stop
ros2 daemon start
```

Use the same domain in every terminal.

### `joint_motion_generator_velocity_discontinuity`

This means the robot rejected motion as too discontinuous. Use smaller target
changes, confirm the current measured joints before publishing, and keep the
conservative velocity and acceleration limits.

### `franka_robot_state_broadcaster` lock errors

This is state publishing struggling to lock its realtime publisher. It is
separate from the Isaac command controller. Avoid echoing high-rate heavy topics
while testing. If needed, tune:

```yaml
franka_robot_state_broadcaster:
  ros__parameters:
    lock_try_count: 10
    lock_sleep_interval: 10
    lock_log_error: true
    lock_update_success: true
```

## How the Controller Was Created

The controller was added inside `franka_example_controllers` so it follows the
existing Franka ROS 2 controller plugin pattern.

Main files:

```text
src/franka_example_controllers/include/franka_example_controllers/isaac_joint_stream_controller.hpp
src/franka_example_controllers/src/isaac_joint_stream_controller.cpp
src/franka_example_controllers/CMakeLists.txt
src/franka_example_controllers/franka_example_controllers.xml
src/franka_bringup/config/controllers.yaml
```

Implementation outline:

1. A new class `IsaacJointStreamController` was added, derived from
   `controller_interface::ControllerInterface`.
2. It claims the seven FR3 position command interfaces:

   ```text
   fr3_joint1/position
   fr3_joint2/position
   fr3_joint3/position
   fr3_joint4/position
   fr3_joint5/position
   fr3_joint6/position
   fr3_joint7/position
   ```

3. It reads the seven FR3 position state interfaces.
4. On first update after activation, it initializes `desired_q` and `target_q`
   from the current measured joint positions.
5. It subscribes to `/isaac_joint_targets` using `sensor_msgs/msg/JointState`.
6. The subscriber validates and remaps incoming commands by joint name.
7. The update loop rejects stale, incomplete, too-large, and out-of-limit targets.
8. The update loop generates smooth commands using velocity and acceleration
   limits, then writes the filtered `desired_q` to the position command interfaces.
9. The controller was registered in `franka_example_controllers.xml` as:

   ```text
   franka_example_controllers/IsaacJointStreamController
   ```

10. The source file and `sensor_msgs` dependency were added in `CMakeLists.txt`.
11. `franka_bringup/config/controllers.yaml` was updated with the controller
    type and conservative parameters.

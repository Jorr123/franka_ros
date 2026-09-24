# FR3 Continuous Cartesian Keyboard Servo

This is a separate streaming-control path. It does not replace or modify the
working Pilz LIN keyboard jog.

## Architecture

```text
keyboard repeat at 20 Hz
  -> base-frame Cartesian velocity
  -> /servo_node/delta_twist_cmds
  -> MoveIt Servo collision/singularity/joint-limit handling
  -> short JointTrajectory messages
  -> /fr3_arm_controller/joint_trajectory
  -> existing effort-backed joint trajectory controller
  -> FR3
```

The optional pilot recorder integrates those velocities into relative
Cartesian deltas at 10 Hz. With `--enable-gripper`, verified open/grasp events
are also recorded. Camera frames are not represented yet.

The keyboard node also observes both sides of the downstream control boundary:

```text
/fr3_arm_controller/joint_trajectory
  -> what MoveIt Servo sent

/fr3_arm_controller/controller_state
  -> controller desired, measured, and error states
```

A sustained command is halted when Servo reports no warning but measured TCP
progress remains below the configured diagnostic threshold. This guard does not
change Servo, controller, or Franka parameters.

## Install And Build

MoveIt Servo is an additional Humble package:

```bash
sudo apt-get install ros-humble-moveit-servo

cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select franka_fr3_moveit_config --symlink-install
source install/setup.bash
```

Do not rebuild while the real-robot launch is running.

## Test 1: Keyboard Dry Run

This test requires no ROS launch and cannot move the robot:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/fr3_keyboard_cartesian_servo.py \
  --max-session-seconds 30
```

Select `X`, `Y`, or `Z`, then press or hold Left/Right. Terminal key-repeat
refreshes the command. Releasing the arrow forces zero velocity after 0.25 s.
Space forces zero immediately and `Q` exits.

To test the O/G key handling without ROS or robot motion:

```bash
python3 scripts/fr3_keyboard_cartesian_servo.py \
  --enable-gripper \
  --gripper-grasp-width 0.05 \
  --gripper-force 30 \
  --max-session-seconds 30
```

Both gripper keys halt Cartesian output before handling the gripper command.
`O` opens to `--gripper-open-width`; `G` grasps at the configured width and
force.

## Test 2: Fake Hardware

Terminal 1:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=127.0.0.1 \
  use_fake_hardware:=true \
  load_gripper:=true
```

Terminal 2:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch franka_fr3_moveit_config servo.launch.py \
  robot_ip:=127.0.0.1 \
  use_fake_hardware:=true \
  load_gripper:=true
```

Terminal 3:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/fr3_keyboard_cartesian_servo.py \
  --execute \
  --max-session-seconds 30 \
  --max-cumulative-translation 0.05
```

Type the exact one-time `START SERVO` confirmation. Begin in fine mode and use
short key presses. Confirm that releasing the key and Space both stop motion.

Both launches must use the same robot-description arguments. In particular,
`fr3_hand_tcp` exists only when the hand is loaded. This workspace's fake
effort hardware accepts and exposes Servo's short trajectories but does not
simulate physical effort-driven joint motion. Treat this as a command-wiring
and halt-behavior test; inspect the command stream with:

```bash
ros2 topic echo /fr3_arm_controller/joint_trajectory
```

## Pilot Recording

The JSONL recorder is for validating timing and action semantics before camera
integration. It is not yet a complete SmolVLA dataset.

```bash
python3 scripts/fr3_keyboard_cartesian_servo.py \
  --execute \
  --enable-gripper \
  --gripper-open-width 0.08 \
  --gripper-grasp-width 0.05 \
  --gripper-force 30 \
  --task "Pick up the red box" \
  --record-jsonl datasets/fr3_box_pick/pilot_0001/servo_frames.jsonl
```

Each frame contains measured joint state, TCP transform when available,
gripper width when available, Servo status, the relative translation integrated
over the 0.1 s record interval, any verified open/grasp event, the latest Servo
trajectory output, and the controller's desired, actual, and error values.

## Bounded Downstream Tracking Diagnostic

Use this test to distinguish a Servo calculation problem from downstream
controller tracking. Start the normal real FR3 MoveIt launch in terminal 1 and
the standalone Servo launch in terminal 2. Do not leave an old Servo node
running after MoveIt has stopped.

Terminal 1:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=172.16.0.2 \
  load_gripper:=true
```

Terminal 2:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch franka_fr3_moveit_config servo.launch.py \
  robot_ip:=172.16.0.2 \
  load_gripper:=true
```

With the workspace clear and the Franka user stop within reach, terminal 3:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 scripts/fr3_keyboard_cartesian_servo.py \
  --execute \
  --fine-speed 0.005 \
  --normal-speed 0.010 \
  --coarse-speed 0.020 \
  --max-linear-speed 0.020 \
  --key-timeout 0.60 \
  --max-session-seconds 15 \
  --max-cumulative-translation 0.010 \
  --record-rate 20 \
  --record-jsonl /tmp/fr3_servo_tracking_x.jsonl \
  --task "Bounded X-axis Servo tracking diagnostic"
```

Type the exact `START SERVO` confirmation, select `X`, remain in fine mode, and
hold one arrow for about 1.2 seconds. The default diagnostic requires at least
10 percent measured progress after one second and 3 mm of integrated command.
Failure immediately publishes halt messages and preserves the JSONL log.

Interpret the final report as follows:

```text
Servo output endpoint changes, controller desired changes, actual TCP does not
  -> downstream trajectory-controller tracking problem

Servo output does not change
  -> Servo command, kinematics, or output configuration problem

Controller actual joints change but TCP moves in the wrong direction
  -> frame, kinematics, or FK/TF interpretation problem

Servo reports collision/singularity deceleration
  -> expected Servo safety scaling; the downstream progress guard is deferred
```

## Real Robot Gate

Real continuous-arm execution is currently on hold. The experimental direct
position-command profile triggered libfranka velocity/acceleration continuity
reflexes and has been removed. Fake hardware cannot reproduce that hardware
protection. Other than the explicitly bounded diagnostic above, keep testing
this combined keyboard and gripper interface in dry-run or fake-hardware mode
until a rate-limited real-time arm controller is validated.

Defaults are deliberately slow:

```text
fine:               0.005 m/s
normal:             0.010 m/s
coarse:             0.020 m/s
command rate:       20 Hz
record rate:        10 Hz
command watchdog:   0.25 s
session duration:   120 s
cumulative motion:  0.50 m
```

MoveIt Servo slowing or halting for collision, singularity, or a joint bound is
not overridden. The Franka user stop remains the physical emergency stop.

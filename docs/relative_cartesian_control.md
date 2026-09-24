# Guarded Relative Cartesian Control for the FR3

## Purpose

`scripts/moveit_cartesian_delta_test.py` commands one small end-effector
translation through the existing real-robot stack:

```text
relative XYZ command
  -> MoveIt Cartesian path
  -> timed joint trajectory
  -> ros2_control
  -> franka_hardware
  -> libfranka
  -> real FR3
```

The command is relative to the end effector's current measured pose. `dx`,
`dy`, and `dz` are expressed in MoveIt's `base` planning frame. Orientation is
held constant.

This is not direct `libfranka` control and does not bypass MoveIt or
`ros2_control`.

## Safety Behavior

The script is plan-only unless `--execute` is explicitly supplied. It:

1. Reads all seven real FR3 positions and velocities from `/joint_states`.
2. Refuses to plan while the robot is moving.
3. Reads the current `base -> fr3_hand_tcp` transform.
4. Builds one relative XYZ target while preserving orientation.
5. Requests a collision-aware straight Cartesian path from MoveIt.
6. Requires at least 99.9 percent of the path.
7. Uses MoveIt's time-parameterized result and slows it only as much as needed
   to enforce conservative absolute joint velocity and acceleration limits. It
   never invents timestamps for an untimed path.
8. Rejects an unexpected start state, large joint excursion, adjacent joint
   jump, excessive velocity, or excessive acceleration.
9. Rechecks the measured state immediately before execution.
10. After execution, verifies Cartesian position, orientation, final joint
    positions, and settled joint velocity.

Any failed gate returns a nonzero exit status and blocks or stops the command.
The script does not attempt an automatic corrective move.

## Build Requirement

No `colcon build` is required for these Python files. Source ROS and the
existing built workspace in every terminal:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 1. Start the Real-Robot MoveIt Stack

Use one terminal:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=172.16.0.2
```

Do not separately launch `franka_bringup franka.launch.py` at the same time.
The MoveIt launch starts the required hardware and controller stack.

Keep the workspace clear and the Franka user stop within reach.

## 2. First Plan-Only Test

Open a second sourced terminal. Request 5 mm upward in the `base` frame:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 scripts/moveit_cartesian_delta_test.py --dz 0.005
```

This cannot move the robot. A usable result must include:

```text
Mode: PLAN ONLY
Collision checking: ENABLED
Cartesian path fraction: 1.000000
Planning result: SUCCESS
Cartesian trajectory safety check: PASS
Plan-only run complete.
```

Inspect the displayed trajectory in RViz. The tool center point should follow a
short straight line and the arm should not wrap around or change posture
dramatically.

Do not execute if:

- the path fraction is below `0.999`;
- any safety check fails;
- the preview contains a large arm swing;
- the robot is near an obstacle or singular posture; or
- the requested base-frame direction is not the direction you intended.

## 3. Execute the Same Small Direction

Only after the plan-only result looks correct:

```bash
python3 scripts/moveit_cartesian_delta_test.py --dz 0.005 --execute
```

A fully successful real motion ends with:

```text
Execution result: SUCCESS
Final result: VERIFIED CARTESIAN SUCCESS
```

The script checks measured state after MoveIt returns. Treat any verification
failure as a stopped experiment and inspect the robot and MoveIt logs before
sending another command.

For a 5 mm command, first confirm that MoveIt's log reports at least 10
interpolated Cartesian points. The safety report may contain fewer timed
trajectory points because time parameterization can combine intermediate
samples. Also confirm:

```text
Maximum planned velocity:  no more than 0.020000 rad/s
Maximum planned accel:     no more than 0.050000 rad/s^2
```

The exact duration depends on the robot posture. A duration near one or two
seconds is reasonable for this small test. A duration near eight seconds and a
`33.333x` slowdown indicates the obsolete blanket-scaling behavior; stop and
confirm that the current workspace was rebuilt, sourced, and relaunched.

## 4. Test the Opposite Direction

Plan first:

```bash
python3 scripts/moveit_cartesian_delta_test.py --dz -0.005
```

After inspecting the RViz path:

```bash
python3 scripts/moveit_cartesian_delta_test.py --dz -0.005 --execute
```

This is a useful first positive/negative pair because it should approximately
return the end effector to its previous height.

## 5. Test X and Y Separately

Plan only:

```bash
python3 scripts/moveit_cartesian_delta_test.py --dx 0.005
python3 scripts/moveit_cartesian_delta_test.py --dy 0.005
```

Execute only one checked direction at a time:

```bash
python3 scripts/moveit_cartesian_delta_test.py --dx 0.005 --execute
python3 scripts/moveit_cartesian_delta_test.py --dy 0.005 --execute
```

The signs refer to the axes of the `base` frame, not image-left/image-right and
not the gripper's local axes. Use the RViz axes to confirm their physical
meaning.

## Conservative Defaults

```text
maximum Cartesian command:       0.020 m
Cartesian interpolation step:    0.0005 m
maximum planned joint velocity:  0.020 rad/s
maximum planned joint accel:     0.050 rad/s^2
maximum joint excursion:         0.10 rad
maximum adjacent joint step:     0.03 rad
Cartesian verification error:    0.005 m
joint verification error:        0.010 rad
maximum settled velocity:        0.010 rad/s
```

The first physical test should remain `0.005 m`. Do not raise limits merely to
force a rejected path to execute; a rejection can indicate a poor posture,
nearby singularity, collision, stale state, or unexpected IK branch.

## How This Fits the VLA Pipeline

This script is the standalone Cartesian motion checkpoint. Once small positive
and negative X/Y/Z tests pass repeatedly, the next adapter can accept a bounded
policy action such as:

```json
{
  "action": "cartesian_delta",
  "translation": {
    "x": 0.002,
    "y": 0.000,
    "z": 0.000
  },
  "frame": "base",
  "confidence": 0.91
}
```

Ubuntu should validate that action and call the same guarded Cartesian planning
logic. The VLA should not send a raw joint trajectory and should never send
commands directly to `libfranka`.

This first version intentionally supports translation only. Small relative
orientation deltas and gripper commands should be added as separate, bounded
action fields after XYZ translation is verified on the real robot.

## Troubleshooting

### `Cartesian path is incomplete`

MoveIt could not follow the whole straight line while satisfying IK, jump, and
collision checks. Try a smaller translation or move to a better starting
posture through a separately reviewed MoveIt joint-space plan.

### `Unsafe Cartesian trajectory`

The Cartesian target caused suspicious joint-space behavior or exceeded a
configured dynamic limit. Inspect the reported joint and RViz path. Do not
immediately increase the failing limit.

### `MoveIt returned an untimed Cartesian trajectory`

Execution is intentionally blocked. Do not add hand-written per-point
timestamps. Check that the normal FR3 MoveIt `move_group` node and Cartesian
path service are being used.

### `Pre-execution measured-state check failed`

The robot moved after planning or the plan no longer starts at the measured
state. Run the plan again from the new current state.

### `CARTESIAN VERIFICATION FAILED`

MoveIt completed execution, but the measured robot pose or joint state did not
meet the required result. Do not automatically retry. Check the reported target
error, joint error, settled velocity, and robot/controller logs.

### `TIMED_OUT` or `CONTROL_FAILED`

Do not send another Cartesian command until the controller log is checked.
`CONTROL_FAILED` can mean the real joints did not reach their configured goal
tolerances. `TIMED_OUT` can instead be MoveIt cancelling before the controller
returns its more precise tolerance result.

After a controller failure, the script still reads the measured final pose and
prints verification details for diagnosis. That diagnostic measurement never
changes the failed command into a success, even when the measured endpoint is
close to the target.

The guarded tool limits the retimed trajectory to `0.020 rad/s` and
`0.050 rad/s^2`. The MoveIt execution margin is also longer than the
controller's goal-time tolerance so that future failures should be reported as
controller failures rather than ambiguous MoveIt timeouts. Do not loosen the
joint goal tolerances merely to suppress these errors.

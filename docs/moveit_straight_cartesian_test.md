# Guarded Straight Cartesian Test

`scripts/moveit_cartesian_ik_joint_test.py` defaults to MoveIt's Cartesian
interpolation service. A second guarded straight-line option is available
through the official Pilz industrial motion planner:

```text
--path-mode pilz-lin
```

The endpoint-only RRT path remains available only with
`--path-mode joint-space`. There is no automatic fallback between modes.

The default path:

1. Reads the measured FR3 joints and `base -> fr3_hand_tcp` transform.
2. Requests collision-checked Cartesian interpolation from MoveIt.
3. Constrains joints 1-6 to the measured position plus or minus 0.12 rad.
4. Constrains joint 7 to the measured position plus or minus 0.002 rad.
5. Limits Cartesian interpolation steps to 0.00025 m.
6. Uses MoveIt's 0.10 rad absolute jump detector and disables its relative
   jump detector, which requires too many samples for short micro-motions.
7. Limits trajectory velocity to 0.02 rad/s and acceleration to 0.05 rad/s^2.
8. Calls MoveIt FK at every trajectory point and at interpolated samples no
   more than 0.002 rad apart in joint space.
9. Blocks execution if the planned TCP leaves the requested line, moves
   backward, changes orientation excessively, or misses the endpoint.
10. Rechecks the measured robot state immediately before execution.
11. Verifies the measured joints and TCP after execution.

## Install And Build

Install the Pilz MoveIt plugin once:

```bash
sudo apt-get install ros-humble-pilz-industrial-motion-planner
```

The MoveIt package now includes new planner and limit configuration files, so
build that package once:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select franka_fr3_moveit_config --symlink-install
source install/setup.bash
```

This workspace has already been rebuilt after adding the files. The command is
only needed again if those configuration files are changed.

Stop any old MoveIt launch and restart it so `move_group` loads the new
pipeline:

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=172.16.0.2
```

## First Test: Plan Only

Start the real-robot MoveIt launch in its own terminal. In another terminal:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 scripts/moveit_cartesian_ik_joint_test.py \
  --dy -0.005
```

Do not add `--execute` on the first run.

Required output:

```text
MoveIt path type: STRAIGHT CARTESIAN INTERPOLATION
Cartesian path fraction: 1.000000
Planning result: SUCCESS
Joint-space trajectory safety check: PASS
Planned TCP path FK check: PASS
```

For a pure Y command, inspect these values:

```text
Maximum cross-track error
Maximum vertical line error
Lowest Z offset from start
Planned endpoint error
Maximum orientation error
```

The default cross-track limit is 0.001 m. A planned downward deviation larger
than this blocks execution. Inspect the path in RViz even when all checks pass.

## Second Test: One Execution

Keep the workspace clear and the Franka user stop within reach. Execute one
small command:

```bash
python3 scripts/moveit_cartesian_ik_joint_test.py \
  --dy -0.005 \
  --execute
```

A fully accepted run ends with:

```text
Execution result: SUCCESS
Final result: VERIFIED CARTESIAN SUCCESS
Overall result: VERIFIED STRAIGHT CARTESIAN SUCCESS
```

If execution or measured verification fails, do not send another command.
Save the complete output for diagnosis.

## Incomplete Cartesian Interpolation

MoveIt can report `SUCCESS` with a fraction below `1.0`. This means it found
only the beginning of the requested Cartesian path. The script discards that
partial trajectory and blocks execution.

If Cartesian interpolation returns an incomplete fraction, use Pilz `LIN`
without `--execute`:

```bash
python3 scripts/moveit_cartesian_ik_joint_test.py \
  --path-mode pilz-lin \
  --dy -0.005
```

This mode:

1. Sends the current TCP pose and translated target pose to MoveIt.
2. Selects `pilz_industrial_motion_planner` and its `LIN` command.
3. Verifies measured joint velocity is low, then encodes the exactly
   zero-velocity start state required by Pilz `LIN`.
4. Uses MoveIt for collision checking and trajectory generation.
5. Applies conservative joint and Cartesian limits from the MoveIt config.
6. Independently limits trajectory joint velocity and acceleration.
7. Checks every dense joint sample through MoveIt's
   `/check_state_validity` collision service.
8. Independently calls MoveIt FK on the same dense samples.
9. Rejects line deviation, backward motion, endpoint overrun, orientation
   error, joint jumps, excess velocity, or excess acceleration.

Pilz is not given a joint-7 path constraint. After planning, the script
measures joint 7 across the returned trajectory and rejects the trajectory when
the configured excursion is exceeded.

Required plan-only ending:

```text
MoveIt path type: PILZ INDUSTRIAL LIN
Planning result: SUCCESS
Joint-space trajectory safety check: PASS
Dense collision-state check: PASS
Planned TCP path FK check: PASS
Maximum TCP line deviation
Final Cartesian target error
Final orientation error
Maximum joint 7 excursion
Plan-only run complete. MoveIt FK verified the Pilz LIN trajectory against the requested TCP line.
```

If Pilz fails, the script prints both the symbolic and numeric MoveIt error
code, then stops. It does not call joint-space planning or another planner.

Only after those checks pass and the RViz path looks correct, run one
execution:

```bash
python3 scripts/moveit_cartesian_ik_joint_test.py \
  --path-mode pilz-lin \
  --dy -0.005 \
  --execute
```

A fully verified run ends with:

```text
Overall result: VERIFIED PILZ LIN CARTESIAN SUCCESS
```

## Reverse Direction

After one successful negative-Y execution, plan the reverse direction before
executing it:

```bash
python3 scripts/moveit_cartesian_ik_joint_test.py --dy 0.005
python3 scripts/moveit_cartesian_ik_joint_test.py --dy 0.005 --execute
```

## Diagnostic Legacy Mode

The old endpoint-only joint-space planner can still be inspected with:

```bash
python3 scripts/moveit_cartesian_ik_joint_test.py \
  --path-mode joint-space \
  --dy -0.005
```

This mode does not guarantee a straight TCP path and should not be used to
work around a rejected straight Cartesian plan.

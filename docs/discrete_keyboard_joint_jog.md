# FR3 Discrete Keyboard Joint Jog

This tool accepts one keyboard action at a time and uses the existing guarded
MoveIt path:

```text
keyboard -> MoveIt plan -> optional confirmation -> exact trajectory execution
         -> fr3_arm_controller -> ros2_control -> libfranka -> FR3
         -> measured joint-state verification
```

It does not stream velocity commands and does not use MoveIt Servo. Only one
joint can be changed by one key action.

## Safety behavior

- Plan-only is the default.
- Real execution requires the `--execute` argument.
- Normal live mode requires an empty `Enter` confirmation for every plan.
- Explicit `--auto-execute` mode removes the per-jog confirmation.
- A fresh measured state is read before planning and again before execution.
- Execution is blocked if the robot moved or is not settled after planning.
- The exact displayed trajectory is sent after confirmation; it is not replanned.
- A failed plan, execution, or measured verification terminates the session.
- Commands cannot be queued while planning, executing, or verifying.
- In auto-execute mode, keys pressed while a jog is active are discarded.
- The default session allows at most 20 verified commands and 0.20 rad of
  cumulative requested movement.

`Space` exits only while the program is waiting for a key. It is not an
emergency stop during execution. Keep the Franka user stop within reach.

## Controls

```text
1-7         select fr3_joint1 through fr3_joint7
Left        plan a negative joint delta
Right       plan a positive joint delta
F           fine increment, 0.010 rad
N           normal increment, 0.030 rad
C           coarse increment, 0.060 rad
Space or Q  end the idle keyboard session
H           print help
Enter       confirm a successfully planned live movement
```

## Start MoveIt

Use the isolated ROS domain in every Ubuntu ROS terminal:

```bash
cd ~/franka_ros2_ws
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2
```

Do not run the bounded VLA supervisor or another robot command source at the
same time.

## Verify the ROS graph

In another terminal with the same environment:

```bash
cd ~/franka_ros2_ws
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 control list_controllers
ros2 topic echo /franka/joint_states --once
```

Confirm that `fr3_arm_controller` is active and the state message contains
`fr3_joint1` through `fr3_joint7`.

## Plan-only test

```bash
python3 scripts/fr3_keyboard_joint_jog.py
```

Select a joint and press an arrow key. MoveIt should report a successful plan,
but the robot cannot move in this mode.

## Real execution test

Clear the robot workspace and keep the user stop accessible, then run:

```bash
python3 scripts/fr3_keyboard_joint_jog.py --execute
```

The default MoveIt velocity and acceleration scaling values are both `0.20`.
Start in fine mode. Press `F`, select a joint with `1-7`, and press one arrow
key. Inspect the target and plan. Press `Enter` on an empty line only when the
movement is appropriate. Type anything else to cancel.

A completed movement must finish with both:

```text
Execution result: SUCCESS
Final result: VERIFIED SUCCESS
```

## Repeated keyboard jogging without per-move confirmation

After plan-only and confirmed execution have both passed, start the bounded
auto-execute mode with:

```bash
python3 scripts/fr3_keyboard_joint_jog.py \
  --execute \
  --auto-execute \
  --max-commands 20 \
  --max-cumulative-delta 0.20
```

Press `F` and select a joint with `1-7`. Each fresh Left or Right arrow press
then plans, checks, executes, and verifies one `0.010 rad` jog without asking
for Enter. Wait until `Jog verified` appears before pressing the next arrow.
Holding an arrow is not continuous velocity control and is intentionally not
supported. `Space` or `Q` exits when the tool is waiting for a key.

After any fault, controller failure, Franka reflex, missing state, or failed
verification, do not restart the keyboard tool until the robot and MoveIt logs
have been checked.

No `colcon build` is required because this is a workspace-level Python tool.

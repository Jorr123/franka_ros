# FR3 Discrete Keyboard Cartesian Jog

This tool converts one keyboard arrow press into one guarded relative Cartesian
translation:

```text
keyboard
  -> moveit_cartesian_ik_joint_test.py --path-mode pilz-lin
  -> MoveIt Pilz LIN planning
  -> collision, dynamics, FK-line, endpoint, and orientation validation
  -> optional execution through fr3_arm_controller
  -> measured joint and TCP verification
```

It does not stream velocity commands. It starts a new, complete planning and
verification cycle for each arrow press. Orientation remains unchanged and the
translation is expressed in the `base` frame.

## Controls

```text
X/Y/Z       select the base-frame translation axis
Left        move in the negative direction
Right       move in the positive direction
F           fine increment, 0.010 m
N           normal increment, 0.025 m
C           coarse increment, 0.050 m
O           open the gripper to its configured width
G           grasp using the configured width and force
Space or Q  end the idle keyboard session
H           print help
Enter       approve a live movement when auto-execute is disabled
```

`Space` is not an emergency stop while a child planning or execution process is
active. Keep the Franka user stop within reach.

## Start MoveIt

Use the same isolated ROS environment and MoveIt launch that already passes the
standalone Pilz LIN test. Do not run another robot command source concurrently.

```bash
cd ~/franka_ros2_ws
export ROS_DOMAIN_ID=99
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2
```

In a second terminal, source the same environment.

## Plan-only test

```bash
python3 scripts/fr3_keyboard_cartesian_jog.py
```

Press `F`, select `Y`, and press Left or Right once. The child Cartesian tool
must finish all plan validations, but the robot cannot move because `--execute`
was not supplied.

## Enable the gripper

Gripper controls are opt-in and require an explicit expected grasp width and
force:

```bash
python3 scripts/fr3_keyboard_cartesian_jog.py \
  --enable-gripper \
  --gripper-open-width 0.07 \
  --gripper-grasp-width 0.035 \
  --gripper-speed 0.02 \
  --gripper-force 30
```

This is still plan-only. `O` and `G` only print the proposed command. After the
configuration has been checked, add `--execute`. A grasp succeeds only when an
object is present and the measured contact width lies within the configured
grasp interval, which defaults to `0.035 +/- 0.005 m` in this example.

## Confirmed execution

After plan-only succeeds, clear the workspace and run:

```bash
python3 scripts/fr3_keyboard_cartesian_jog.py --execute
```

Start with a fine 10 mm move. The Enter prompt approves the command intent; the
child script then obtains a fresh state and independently blocks execution
unless the exact Pilz LIN trajectory passes all of its checks.

A successful live jog ends with the child program's verified Cartesian success
followed by:

```text
Cartesian jog verified. The next key command is now enabled.
```

## Repeated jogging

Only after plan-only and confirmed execution both pass:

```bash
python3 scripts/fr3_keyboard_cartesian_jog.py \
  --execute \
  --auto-execute \
  --max-commands 20 \
  --max-cumulative-distance 0.20
```

Each fresh arrow press requests one complete move. Keys entered while a move is
active are discarded afterward; holding an arrow is not continuous control.
Any planning, controller, Franka, or measured-verification failure terminates
the session without retrying.

The default cumulative translation budget is `0.20 m`. For a deliberately
longer supervised session, disable only that application-level budget while
retaining the command-count cap and all per-motion checks:

```bash
python3 scripts/fr3_keyboard_cartesian_jog.py \
  --execute \
  --no-cumulative-distance-limit \
  --max-commands 50
```

The defaults use velocity and acceleration scaling `0.20`, maximum 50 mm per
keypress, maximum joint-7 excursion `0.05 rad`, maximum per-joint excursion
`0.12 rad`, and maximum total joint excursion `0.30 rad`. These are
application-level guards and can be overridden explicitly, but a rejected plan
should be inspected rather than automatically relaxed.

No `colcon build` is required because this is a workspace-level Python script.

# Cosmos policy control of the standard Franka Hand

Gripper control is disabled by default. It requires `--enable-gripper` on
Windows and on Ubuntu. Arm-only commands keep their existing behavior.
This integration does not send any hardware commands during installation.

## What the policy controls

The eighth DROID channel is treated as an open/close signal:

- At or below 0.3: request open.
- At or above 0.7: request grasp.
- Between those thresholds: hold the current state.

These are adapter hysteresis thresholds, not new model training settings.
Raw finite predictions are preserved without clipping. A Hand measured at
75 mm or wider is treated as open. A narrower width is treated as a grasp
only after this Ubuntu receiver has verified a grasp at that width. A partly
closed hand on startup is not assumed to hold an object. Windows reads this
receiver-local confirmation through a read-only health request, and Ubuntu
rejects forged/stale confirmation. An already satisfied state does not cause
repeated open/grasp commands. Restarting Ubuntu clears the confirmation.

With `--actions-per-chunk K`, Windows selects at most K actions, ending at
the first gripper transition. Ubuntu executes that unchanged arm prefix,
waits for completion/settling, then performs the gripper event at its endpoint.
The next cycle captures new state and images. Gripper events do not occur
concurrently with arm motion, and changes later in the discarded horizon
are not replayed. This may shorten a nominal 12-action chunk to one action.

Opening uses `/franka_gripper/move` to 0.08 m. Grasping uses
`/franka_gripper/grasp`, with operator-configured object contact width and
force. The model cannot set speed, force, epsilon or arbitrary gripper travel.
These ROS actions and measured joint states must already be available from
the existing robot launch. No automatic homing or error recovery is sent.

## Previous test settings

The repository contains examples of 35 mm / 30 N in
`discrete_keyboard_cartesian_jog.md`, and 50 mm / 30 N in
`fr3_continuous_cartesian_servo.md`. They are example configurations. No saved
hardware result in this checkout establishes which fits the current green box.
Measure the contact width where the fingers will grasp, and establish the
appropriate force using the existing standalone gripper test before enabling
automatic grasping. Do not treat a unit test or documentation example as a
record of a successful physical grasp.

## Review a plan without moving

Keep Robot + Read State and Cosmos running. Stop the old Cosmos receiver.
In a ROS-configured Ubuntu terminal, the following uses the **35 mm / 30 N
example solely to inspect configuration and plans**; it enables no execution:

```bash
cd ~/franka_ros2_ws
git pull --ff-only
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=99

python3 scripts/fr3_cosmos_action_server.py \
  --enable-gripper --gripper-grasp-width 0.035 --gripper-force 30
```

In Windows PowerShell, using the existing runtime and Python variables:

```powershell
& $python "$runtime\inference\run_cosmos3.py" plan-once `
  --gripper-open-width-m 0.08 --prompt "pick up the green box" `
  --actions-per-chunk 12 --enable-gripper
```

Neither the plan endpoint nor the above startup configuration executes arm
or gripper commands. The returned `plan.gripper` shows the frozen operation,
width, speed, configured force and epsilon. The hash binds both arm and
gripper plans. An incompatible or disabled receiver rejects the request.

## Live operation after the object settings are established

Set `GRIPPER_CONTACT_WIDTH_M` and `GRIPPER_FORCE_N` to your tested values.
In the same Ubuntu ROS terminal:

```bash
: "${GRIPPER_CONTACT_WIDTH_M:?Set the tested object contact width in metres}"
: "${GRIPPER_FORCE_N:?Set the tested grasp force in newtons}"
python3 scripts/fr3_cosmos_action_server.py \
  --allow-auto-executions 15 --verification-mode report-only \
  --enable-gripper \
  --gripper-grasp-width "$GRIPPER_CONTACT_WIDTH_M" \
  --gripper-force "$GRIPPER_FORCE_N"
```

Contact width must be above 0.005 m and below 0.075 m. Force is explicitly
required within the existing helper's 30..70 N bounds. Speed defaults to and
is capped at 0.02 m/s. Grasp epsilon is 0.005 m on each side; opening tolerance
is 0.005 m. These bounds are configuration constraints, not an assurance that
a particular object tolerates the selected force.

First run `execute-once` with `--enable-gripper` for one supervised cycle.
Then, with a clear workspace and stop within reach, the loop command is:

```powershell
& $python "$runtime\inference\run_cosmos3.py" execute-loop `
  --gripper-open-width-m 0.08 --prompt "pick up the green box" `
  --actions-per-chunk 12 --num-chunks 15 --max-duration-seconds 900 `
  --enable-gripper
```

This can physically move the arm and open/close the fingers without another
confirmation. One arm-prefix/gripper-event pair consumes one execution attempt.

## Checks and outcomes

Ubuntu checks action-server availability before arm execution. It checks a
dense finger-width sweep against the whole MoveIt scene at planning, and
again at the actual measured arm endpoint before the gripper action. Checks
use both finger joints with at most 1 mm width spacing. Known scene obstacles
can reject a grasp: intentional target-object contact requires a separately
reviewed planning-scene/contact configuration; no collisions are silently
allowed. Scene checks depend on the scene being complete and current.

Arm controller failures prevent gripper commands. After the arm phase,
fresh independent state, settled velocities and final pose are checked before
the fingers move. Gripper action success is followed by another measured
sample, width verification, independent-state agreement and settled-arm checks.
An empty closure outside the configured contact interval fails.

Arm `report-only` accuracy warnings may permit the gripper event. Gripper
failures, timeouts, state disagreement or finger collisions are always faults,
discard remaining execution attempts and stop the loop. A fault after the
arm phase preserves `execution.arm_result`; it does not imply that the arm
stayed still. Lost/timeout outcomes are never automatically retried.

Receipts contain `execution.gripper`, including operation, measured width and
whether a command was sent. Force fields are commanded settings, not force
sensor measurements. `PASSED` confirms the action/width checks; it does not
prove an object was lifted or that the pickup task succeeded. Grasp/lift success
detection remains future work. On interruption, an in-flight arm or gripper
action may still complete; inspect Ubuntu and the robot before another run.

The implementation is tested with simulated actions/state and local sockets.
Live gripper integration still requires physical validation.

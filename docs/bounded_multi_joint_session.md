# Bounded Multi-Joint FR3 Session

## What Changed

The bounded Windows-to-Ubuntu protocol now supports a strict allowlist of
named FR3 joints. Schema version 2 replaces `allowed_joint` with:

```text
allowed_joints
max_moving_joints
max_delta
max_total_delta
max_steps
max_duration_seconds
max_cumulative_delta
```

The initial test allows only `fr3_joint1` and `fr3_joint3`. Each action may
move one or both of those joints. The Ubuntu supervisor still validates the
action, plans through MoveIt, executes synchronously, and checks the measured
joint state before another action is accepted.

`max_total_delta` and `max_cumulative_delta` use the L1 sum. For example:

```text
abs(+0.01) + abs(-0.01) = 0.02 rad for one step
two verified steps = 0.04 rad cumulative
```

## Files To Copy To Windows

Copy these updated files from `windows_vlm_bridge` to `C:\vlm_bridge`:

```text
fr3_action_protocol.py
fr3_remote_session_control.py
static_vla_execution_session.py
closed_loop_vla_execution_session.py
bounded_joint_action_session.py
actions\two_joint_forward.json
actions\two_joint_reverse.json
```

The old one-joint visual loop remains available, but both computers must use
the updated schema-v2 protocol file.

## 1. Offline Test

On Ubuntu, this runs only local unit tests. It does not contact ROS or the
robot:

```bash
cd ~/franka_ros2_ws
python3 -m unittest discover -s tests -v
```

On Windows, validate the two action files without contacting Ubuntu:

```powershell
cd C:\vlm_bridge
conda activate vlm

python bounded_joint_action_session.py `
  --action-json-file C:\vlm_bridge\actions\two_joint_forward.json `
  --action-json-file C:\vlm_bridge\actions\two_joint_reverse.json `
  --max-moving-joints 2 `
  --max-delta 0.01 `
  --max-total-delta 0.02 `
  --max-cumulative-delta 0.04 `
  --log-file C:\vlm_bridge\session_logs\two_joint_dry.jsonl
```

Required ending:

```text
Status:                    DRY_RUN_COMPLETE
Actions validated:         2
Ubuntu contacted:          False
Robot execution:           False
```

Do not add `--run-armed-session` during this test.

## 2. Start The Real-Robot Stack

Keep the robot workspace clear and the user stop within reach.

Ubuntu terminal 1:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2
```

Ubuntu terminal 2:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export FR3_BRIDGE_SECRET='YOUR_EXISTING_SHARED_SECRET'

python3 scripts/fr3_bounded_session_supervisor.py \
  --enable-bounded-session-execution \
  --enable-remote-session-arm \
  --remote-arm-joint fr3_joint1 \
  --remote-arm-joint fr3_joint3 \
  --bind-host 10.10.10.1 \
  --allowed-client 10.10.10.2 \
  --session-max-steps 2 \
  --session-max-duration 60 \
  --session-max-cumulative-delta 0.04 \
  --max-moving-joints 2 \
  --max-delta 0.01 \
  --max-total-delta 0.02
```

The supervisor must report schema version 2, multi-joint bounded sessions,
and no armed session. Starting it does not move the robot.

## 3. Arm From Windows

Windows PowerShell:

```powershell
cd C:\vlm_bridge
conda activate vlm
$env:FR3_BRIDGE_SECRET = "YOUR_EXISTING_SHARED_SECRET"

python fr3_remote_session_control.py arm `
  --joint fr3_joint1 `
  --joint fr3_joint3 `
  --max-moving-joints 2 `
  --max-steps 2 `
  --max-duration 60 `
  --max-delta 0.01 `
  --max-total-delta 0.02 `
  --max-cumulative-delta 0.04
```

Arming creates temporary authority only. It does not send a trajectory.

## 4. Execute Two Bounded Actions

Review the action files and robot area again. Then run:

```powershell
python bounded_joint_action_session.py `
  --action-json-file C:\vlm_bridge\actions\two_joint_forward.json `
  --action-json-file C:\vlm_bridge\actions\two_joint_reverse.json `
  --max-moving-joints 2 `
  --max-delta 0.01 `
  --max-total-delta 0.02 `
  --max-cumulative-delta 0.04 `
  --max-session-seconds 50 `
  --log-file C:\vlm_bridge\session_logs\two_joint_real_01.jsonl `
  --run-armed-session
```

Expected successful ending:

```text
Status:                    SESSION_COMPLETED
Actions validated:         2
Verified steps:            2
Cumulative proposed/done:  0.040000 rad
Ubuntu contacted:          True
Robot execution:           True
```

The second action is the inverse of the first, so it should approximately
return joints 1 and 3 to their starting positions. A failed plan, execution,
receipt, measured verification, sequence, signature, freshness check, or
budget check ends the session without sending the next action.

## Important Boundaries

- Windows never calls `libfranka` or `ros2_control` directly.
- The supervisor's hard source-code ceiling is `0.10 rad` per joint. The
  documented and tested operating limit remains `0.01 rad` per joint unless a
  separate launch/session configuration deliberately selects a larger value.
- The Windows client is dry-run-only unless `--run-armed-session` is present.
- Remote arming is unavailable unless Ubuntu was explicitly started with
  `--enable-remote-session-arm` and the two permitted joints.
- There is one action in flight and no automatic retry.
- Only a measured `VERIFIED_SUCCESS` step consumes cumulative budget.
- Keep the remote test at `0.01 rad` per joint even though larger local manual
  tests have succeeded.
- Python script changes do not require `colcon build`; restart the running
  supervisor after updating the files.

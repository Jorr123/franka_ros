# Bounded Static-Image Execution Session

This checkpoint turns the tested static-image pipeline into a short, locally
authorized execution session. It still does not use Isaac Sim, a live camera,
or an unbounded control loop.

```text
Ubuntu operator locally arms one session
  -> Windows reads one saved image
  -> Qwen proposes one tiny joint delta
  -> Windows validates and signs numbered step N
  -> Ubuntu validates session, sequence, action, and limits
  -> MoveIt plans from fresh measured joint state
  -> Ubuntu rechecks the robot start state
  -> MoveIt executes through ros2_control
  -> Ubuntu verifies measured /joint_states
  -> only VERIFIED_SUCCESS unlocks step N+1
```

The first real trial should contain exactly two actions on `fr3_joint3`:

```text
right.png -> +0.03 rad
left.png  -> -0.03 rad
```

The two steps are approximately a return motion, not a guarantee that every
other joint will return to its exact original value.

## Safety Boundaries

- Ubuntu creates the session ID and motion authority through a private Unix
  socket with mode `0600`.
- The operator must type `ARM` exactly once on Ubuntu after reviewing the
  displayed limits.
- One session permits one named joint, one action in flight, a fixed maximum
  step count, a deadline, a per-step limit, and a cumulative-motion limit.
- Windows must submit sequence `1`, receive `VERIFIED_SUCCESS`, then submit
  sequence `2`.
- There are no automatic retries. A replay, wrong sequence, wrong joint,
  planning failure, execution failure, or verification failure terminates the
  session.
- The bounded supervisor has no `/v1/actions/execute` endpoint.
- A network stop removes authority for future steps. It cannot guarantee that
  an already executing trajectory stops immediately. Use the Franka user stop
  for any unsafe in-flight motion.
- Saved images are not live robot feedback. This checkpoint proves sequencing
  and safety gating, not visual closed-loop control.

## 1. Run The Tests On Ubuntu

```bash
cd ~/franka_ros2_ws
python3 -m unittest -v \
  tests/test_fr3_action_protocol.py \
  tests/test_bounded_session_supervisor.py \
  tests/test_static_vla_execution_session.py \
  tests/test_fr3_session_control.py
```

These tests do not contact MoveIt, Windows, or the robot.

## 2. Transfer The Windows Files

The Windows coordinator imports the other three files, so transfer all four.

Ubuntu temporary file server:

```bash
cd ~/franka_ros2_ws/windows_vlm_bridge
python3 -m http.server 8081 --bind 10.10.10.1
```

Windows PowerShell:

```powershell
Invoke-WebRequest http://10.10.10.1:8081/fr3_action_protocol.py `
  -OutFile C:\vlm_bridge\fr3_action_protocol.py
Invoke-WebRequest http://10.10.10.1:8081/isaac_qwen_coordinator.py `
  -OutFile C:\vlm_bridge\isaac_qwen_coordinator.py
Invoke-WebRequest http://10.10.10.1:8081/static_vla_shadow_session.py `
  -OutFile C:\vlm_bridge\static_vla_shadow_session.py
Invoke-WebRequest http://10.10.10.1:8081/static_vla_execution_session.py `
  -OutFile C:\vlm_bridge\static_vla_execution_session.py
```

Stop the temporary file server with `Ctrl+C`.

## 3. Start MoveIt On Ubuntu

Ubuntu terminal 1:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2
```

Use the same launch that already passed the manual MoveIt tests. Keep the
workspace clear, keep the Franka user stop within reach, and do not start a
second `ros2_control_node`.

## 4. Start The Bounded Supervisor On Ubuntu

Stop the earlier `fr3_ai_supervisor.py`; both supervisors use port `8080`.

Ubuntu terminal 2:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export FR3_BRIDGE_SECRET='YOUR_EXISTING_SHARED_SECRET'

python3 scripts/fr3_bounded_session_supervisor.py \
  --enable-bounded-session-execution \
  --bind-host 10.10.10.1 \
  --port 8080 \
  --allowed-client 10.10.10.2 \
  --allowed-client 127.0.0.1 \
  --session-max-steps 2 \
  --session-max-duration 60 \
  --session-max-cumulative-delta 0.06 \
  --min-confidence 0.70 \
  --max-delta 0.03 \
  --max-total-delta 0.03 \
  --max-moving-joints 1 \
  --goal-tolerance 0.002 \
  --velocity-scaling 0.03 \
  --acceleration-scaling 0.03 \
  --start-state-tolerance 0.01 \
  --max-start-velocity 0.01 \
  --settle-time 1.0 \
  --verification-tolerance 0.01 \
  --non-commanded-drift-limit 0.01 \
  --max-settled-velocity 0.01
```

Required startup messages include:

```text
FR3 bounded-session supervisor started.
NO SESSION IS ARMED. THE ROBOT CANNOT MOVE FROM REMOTE REQUESTS YET.
Direct remote execution endpoint: NONE
```

Do not arm yet.

## 5. Start Qwen On Windows

Windows terminal 1:

```powershell
cd C:\vlm_bridge
conda activate vlm
python -m uvicorn qwen_vlm_server:app `
  --host 0.0.0.0 `
  --port 8000 `
  --workers 1
```

Use the already tested mapping where `right` produces
`{"fr3_joint3": 0.03}` and `left` produces
`{"fr3_joint3": -0.03}`.

## 6. Run Windows Local Inference First

Windows terminal 2:

```powershell
cd C:\vlm_bridge
conda activate vlm

python static_vla_execution_session.py `
  --image C:\vlm_bridge\right.png `
  --image C:\vlm_bridge\left.png `
  --expected-joint fr3_joint3 `
  --max-images 2 `
  --max-session-seconds 50 `
  --max-delta 0.03 `
  --max-total-delta 0.03 `
  --max-moving-joints 1 `
  --interval 0.5 `
  --log-file C:\vlm_bridge\session_logs\execution_dry_01.jsonl
```

Do not add `--run-armed-session`. Expected output includes:

```text
LOCAL INFERENCE ONLY. Ubuntu will not be contacted and the robot cannot move.
Robot execution:     False
Verified steps:      2
```

Inspect the log and Qwen output. Continue only when the actions are exactly
`fr3_joint3 +0.03` followed by `fr3_joint3 -0.03`.

## 7. Locally Arm Two Steps On Ubuntu

Ubuntu terminal 3:

```bash
cd ~/franka_ros2_ws
python3 scripts/fr3_session_control.py arm \
  --joint fr3_joint3 \
  --max-steps 2 \
  --max-duration 60 \
  --max-delta 0.03 \
  --max-cumulative-delta 0.06
```

Review the displayed limits. Type exactly `ARM` only when the robot and
workspace are ready. Ubuntu generates and records the UUID internally. Windows
discovers the single fresh armed session automatically, so there is no UUID to
copy. The 60-second deadline begins when Ubuntu arms the session, so proceed
directly to the next command.

## 8. Execute The Two-Image Session From Windows

Confirm that Windows uses the same secret as Ubuntu:

```powershell
$env:FR3_BRIDGE_SECRET = "YOUR_EXISTING_SHARED_SECRET"
```

Then run:

```powershell
python static_vla_execution_session.py `
  --image C:\vlm_bridge\right.png `
  --image C:\vlm_bridge\left.png `
  --max-images 2 `
  --max-session-seconds 50 `
  --max-delta 0.03 `
  --max-total-delta 0.03 `
  --max-moving-joints 1 `
  --interval 0.5 `
  --log-file C:\vlm_bridge\session_logs\execution_real_01.jsonl `
  --run-armed-session
```

Expected behavior:

1. Windows discovers the one fresh Ubuntu session and confirms its limits.
2. The first image produces one positive `fr3_joint3` step.
3. Ubuntu plans, executes, and verifies it against measured joint state.
4. Only then does Windows process the second image.
5. The second image produces one negative `fr3_joint3` step.
6. Ubuntu reports `COMPLETED` after two verified actions.

Expected final summary:

```text
Status:              SESSION_COMPLETED
Verified steps:      2
Cumulative delta:    0.060000 rad
Robot execution:     True
```

Stop immediately if the proposed joint/sign differs, the robot motion looks
unsafe, or any result is unclear.

## 9. Inspect Session State And Logs

Ubuntu status for the current session:

```bash
python3 scripts/fr3_session_control.py status
```

Expected:

```text
Status:           COMPLETED
Completed steps:  2/2
Terminal code:    MAX_STEPS_REACHED
```

Ubuntu append-only audit log:

```bash
less session_logs/ubuntu/session_<uuid>.jsonl
```

Windows coordinator log:

```powershell
Get-Content C:\vlm_bridge\session_logs\execution_real_01.jsonl
```

For each action, confirm the order:

```text
ACTION_VALIDATED
STEP_VERIFIED
```

Also confirm that Ubuntu contains `STEP_ACCEPTED`, `PLAN_READY`, and
`STEP_VERIFIED` with matching sequence numbers and request IDs.

## Stop Or Failure Handling

To remove authority from the current session locally:

```bash
python3 scripts/fr3_session_control.py disarm
```

The generated session UUID still appears in status output and audit logs. It is
an internal correlation and replay-protection identifier, not an operator
password. `status` and `disarm` retain an optional `--session-id` argument for
diagnostics.

Stopping the supervisor with `Ctrl+C` also removes the network endpoint. A
session marked `EXPIRED`, `FAULT`, `DISARMED`, `STOPPED`, or `COMPLETED` accepts
no additional actions. Never retry a timed-out step because the outcome may be
ambiguous; inspect the robot and logs, then create a new session.

No `colcon build` is needed for these Python supervisor/coordinator files.

## Checkpoint Result

Pass this checkpoint only when:

- the dry run proposes the expected positive and negative actions;
- the real run executes exactly two actions and no more;
- both actions report `VERIFIED_SUCCESS`;
- the session ends in `COMPLETED`;
- measured verification passes after each action;
- wrong/no-marker input and all tested failures produce no additional motion;
- Ubuntu and Windows logs agree on session ID, sequence, request ID, and action
  hash.

After this passes repeatedly, the next checkpoint is a live-camera **shadow**
loop. Live frames should first run inference and MoveIt planning without robot
execution. Isaac remains the final integration stage.

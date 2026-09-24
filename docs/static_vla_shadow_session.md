# Bounded Static VLA Shadow Session

This checkpoint runs several saved Windows images through Qwen and MoveIt in a
bounded sequence. It tests the control-loop state machine without executing any
trajectory.

```text
ordered Windows images
  -> one Qwen inference
  -> deterministic action validation
  -> cumulative and session-limit checks
  -> one authenticated Ubuntu plan request
  -> wait for PLAN_READY
  -> log the result
  -> next image
```

The session coordinator has no execution option. With `--send-plans`, it also
refuses to proceed unless Ubuntu reports all of the following:

```text
mode: PLAN_ONLY
execution_available: false
remote_execution_available: false
local_approval_available: false
```

## What This Proves

- Images stay on Windows.
- Only one inference and plan request is active at a time.
- Every step receives a unique request ID and ordered frame ID.
- Ubuntu returns the matching request ID and action hash.
- The session stops at its step, time, or cumulative-motion limit.
- `NO_ACTION` stops the session safely by default.
- Every state transition is written to a JSONL log.
- The FR3 does not move.

## 1. Run The Tests On Ubuntu

```bash
cd ~/franka_ros2_ws
python3 -m unittest -v tests/test_static_vla_shadow_session.py
```

Expected:

```text
Ran 9 tests
OK
```

## 2. Transfer The Session Script To Windows

On Ubuntu:

```bash
cd ~/franka_ros2_ws/windows_vlm_bridge
python3 -m http.server 8081 --bind 10.10.10.1
```

On Windows PowerShell:

```powershell
Invoke-WebRequest `
  http://10.10.10.1:8081/static_vla_shadow_session.py `
  -OutFile C:\vlm_bridge\static_vla_shadow_session.py
```

The script uses the existing copies of:

```text
C:\vlm_bridge\isaac_qwen_coordinator.py
C:\vlm_bridge\fr3_action_protocol.py
```

Stop the temporary Ubuntu file server with `Ctrl+C`.

## 3. Start MoveIt On Ubuntu

Ubuntu terminal 1:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2
```

Use the previously tested MoveIt launch. Do not start another
`ros2_control_node`.

## 4. Start The Ubuntu Supervisor In Plan-Only Mode

Stop any supervisor that was started with `--enable-local-execution`.

Ubuntu terminal 2:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export FR3_BRIDGE_SECRET='YOUR_EXISTING_SHARED_SECRET'

python3 scripts/fr3_ai_supervisor.py \
  --bind-host 10.10.10.1 \
  --port 8080 \
  --allowed-client 10.10.10.2 \
  --min-confidence 0.70 \
  --max-delta 0.03 \
  --max-total-delta 0.03 \
  --max-moving-joints 1 \
  --goal-tolerance 0.002 \
  --velocity-scaling 0.03 \
  --acceleration-scaling 0.03
```

Do not include `--enable-local-execution`. Required startup line:

```text
MODE: PLAN ONLY. This process cannot execute robot motion.
```

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

Keep the current tested joint mapping and action step. Do not replace the Qwen
server during this checkpoint.

## 6. Run A Local-Inference Session First

Windows terminal 2:

```powershell
cd C:\vlm_bridge
conda activate vlm

python static_vla_shadow_session.py `
  --image C:\vlm_bridge\right.png `
  --image C:\vlm_bridge\left.png `
  --image C:\vlm_bridge\nothing.png `
  --max-steps 3 `
  --max-session-seconds 120 `
  --max-delta 0.03 `
  --max-total-delta 0.03 `
  --max-moving-joints 1 `
  --max-cumulative-delta 0.06 `
  --interval 0.5 `
  --log-file C:\vlm_bridge\session_logs\inference_01.jsonl
```

There is no `--send-plans`, so Ubuntu is not contacted. Expected ending:

```text
Status:               TASK_COMPLETE_NO_ACTION
Accepted actions:     2
MoveIt plans ready:   0
No-action decisions:  1
Robot execution:      DISABLED
```

The cumulative value depends on the configured Windows action step. For
example, two `0.03 rad` actions produce `0.06 rad` total absolute delta.

## 7. Run The Plan-Only Shadow Session

Use a new log filename because logs are never overwritten:

```powershell
$env:FR3_BRIDGE_SECRET = "YOUR_EXISTING_SHARED_SECRET"

python static_vla_shadow_session.py `
  --image C:\vlm_bridge\right.png `
  --image C:\vlm_bridge\left.png `
  --image C:\vlm_bridge\nothing.png `
  --max-steps 3 `
  --max-session-seconds 120 `
  --max-delta 0.03 `
  --max-total-delta 0.03 `
  --max-moving-joints 1 `
  --max-cumulative-delta 0.06 `
  --interval 0.5 `
  --log-file C:\vlm_bridge\session_logs\planning_01.jsonl `
  --send-plans
```

Expected beginning:

```text
Ubuntu safety gate: PLAN_ONLY confirmed
Robot execution: DISABLED
```

Expected ending:

```text
Status:               TASK_COMPLETE_NO_ACTION
Accepted actions:     2
MoveIt plans ready:   2
No-action decisions:  1
Robot execution:      DISABLED
```

The physical FR3 must remain stationary throughout the session.

## 8. Inspect The Log

```powershell
Get-Content C:\vlm_bridge\session_logs\planning_01.jsonl
```

For each moving image, the order must be:

```text
STEP_STARTED
ACTION_VALIDATED
PLAN_READY
```

The next `STEP_STARTED` must appear only after the previous `PLAN_READY`. The
last image should produce `NO_ACTION`, followed by `SESSION_FINISHED`.

The log contains no image bytes and no shared secret. It records image hashes,
actions, request IDs, action hashes, planning summaries, limits, and outcomes.

## Safe Stop Conditions

```text
TASK_COMPLETE_NO_ACTION   No action was required; normal safe completion.
STEP_LIMIT_REACHED        More images existed than the allowed step count.
SESSION_TIMEOUT           The session time limit was reached.
CUMULATIVE_LIMIT_REACHED  Proposed actions exceeded the cumulative limit.
FAILED                    Inference, validation, networking, or planning failed.
```

There are no automatic retries. A failure stops the sequence immediately.

## Checkpoint

Repeat the planning session at least three times with new log filenames. This
phase passes when:

```text
All right/left observations map correctly.
Every accepted action receives exactly one matching PLAN_READY.
NO_ACTION never contacts Ubuntu for a plan.
No stale, duplicate, or mismatched receipt is accepted.
Every run leaves a complete SESSION_FINISHED log entry.
The real FR3 never moves.
```

After this passes, the next implementation is a locally armed execution
session. Ubuntu will still execute one action at a time and require
post-execution verification before Windows may provide the next image. Isaac
remains outside that phase.

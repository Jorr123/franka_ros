# Static-Image Supervised FR3 Execution

This checkpoint tests the complete Windows-to-Ubuntu path with saved images,
Qwen, MoveIt, and the real FR3. Isaac Sim and Isaac Lab are intentionally not
part of this test.

```text
Windows image
  -> Windows Qwen
  -> bounded joint-delta proposal
  -> authenticated HTTP plan request
  -> Ubuntu validation and MoveIt planning
  -> frozen plan stored temporarily on Ubuntu
  -> local Ubuntu inspection and typed approval
  -> MoveIt execution through ros2_control
  -> measured /joint_states verification
  -> VERIFIED_SUCCESS or terminal FAULT
```

Windows cannot execute a trajectory. The Ubuntu HTTP server exposes only
`/v1/actions/plan`. Execution is available only through a private Unix socket
on the Ubuntu computer.

## Safety Properties

- Local execution is disabled unless the supervisor starts with
  `--enable-local-execution`.
- The approval socket is created with mode `0600`, so it is accessible only to
  the Ubuntu user that started the supervisor.
- A plan expires after a short time.
- A newer valid plan supersedes an older unexecuted plan.
- The request ID and SHA-256 action hash must both match.
- The plan is marked consumed before execution begins and can never be reused.
- Fresh measured joints are compared with the planning start state before
  MoveIt receives the trajectory.
- The robot must also be settled below the configured pre-execution velocity
  limit.
- MoveIt execution is followed by measured joint direction, target error,
  non-commanded drift, and settled-velocity checks.
- Failure is terminal. The supervisor never retries robot motion automatically.

## Files

Ubuntu:

```text
scripts/fr3_ai_supervisor.py
scripts/fr3_supervisor_approve.py
scripts/joint_motion_verification.py
```

Windows:

```text
C:\vlm_bridge\qwen_vlm_server.py
C:\vlm_bridge\isaac_qwen_coordinator.py
C:\vlm_bridge\fr3_action_protocol.py
```

## 1. Run The Offline Tests

On Ubuntu, with no robot command being sent:

```bash
cd ~/franka_ros2_ws
python3 -m unittest -v \
  tests/test_fr3_action_protocol.py \
  tests/test_fr3_supervisor_approval.py \
  tests/test_static_vla_shadow_session.py \
  tests/test_windows_isaac_qwen_coordinator.py \
  tests/test_joint_motion_verification.py
```

Expected ending:

```text
Ran 44 tests
OK
```

These Python scripts do not require a `colcon build`.

## 2. Prepare The Robot Area

Before enabling the FR3:

1. Remove people and loose objects from the workspace.
2. Keep one hand near the user stop.
3. Use only the tested `0.03 rad` single-joint action.
4. Do not run Isaac, another robot command publisher, or another
   `ros2_control_node` during this checkpoint.
5. Start with `right.png`, `left.png`, and `nothing.png` only.

## 3. Start MoveIt On Ubuntu

Ubuntu terminal 1:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2
```

Confirm that MoveIt and the real robot are healthy before continuing:

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

Use the MoveIt controller configuration that already worked in RViz. Do not
start a separate Franka bringup process at the same time.

## 4. Start Qwen On Windows

Windows PowerShell terminal 1:

```powershell
cd C:\vlm_bridge
conda activate vlm
python -m uvicorn qwen_vlm_server:app `
  --host 0.0.0.0 `
  --port 8000 `
  --workers 1
```

Keep this terminal open. In another Windows terminal, confirm:

```powershell
curl.exe http://127.0.0.1:8000/health
```

## 5. Configure The Shared Secret

Use the same existing secret in both terminals. On Windows terminal 2:

```powershell
$env:FR3_BRIDGE_SECRET = "YOUR_EXISTING_SHARED_SECRET"
```

On Ubuntu terminal 2:

```bash
export FR3_BRIDGE_SECRET='YOUR_EXISTING_SHARED_SECRET'
```

Do not store the secret in a source file. Keep the Windows and Ubuntu clocks
synchronized because stale and future requests are rejected.

## 6. Start The Supervised Ubuntu Server

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
  --enable-local-execution \
  --plan-ttl 60 \
  --start-state-tolerance 0.01 \
  --max-start-velocity 0.01 \
  --min-confidence 0.70 \
  --max-delta 0.03 \
  --max-total-delta 0.03 \
  --max-moving-joints 1 \
  --goal-tolerance 0.002 \
  --velocity-scaling 0.03 \
  --acceleration-scaling 0.03 \
  --settle-time 1.0 \
  --verification-tolerance 0.01 \
  --non-commanded-drift-limit 0.01 \
  --max-settled-velocity 0.01
```

The startup output must include:

```text
Remote execution: DISABLED (no HTTP execution endpoint exists).
MODE: SUPERVISED SINGLE STEP
Local approval socket: /tmp/fr3_ai_supervisor_approval.sock (mode 0600)
Frozen plan lifetime: 60.0 s
```

From Windows, check:

```powershell
Invoke-RestMethod http://10.10.10.1:8080/health
```

Expected important fields:

```text
mode: SUPERVISED_SINGLE_STEP
remote_execution_available: False
local_approval_available: True
```

## 7. Request One Plan From Windows

Windows terminal 2:

```powershell
cd C:\vlm_bridge
conda activate vlm
$env:FR3_BRIDGE_SECRET = "YOUR_EXISTING_SHARED_SECRET"

python isaac_qwen_coordinator.py `
  --image C:\vlm_bridge\right.png `
  --min-confidence 0.70 `
  --max-delta 0.03 `
  --max-total-delta 0.03 `
  --max-moving-joints 1 `
  --send-plan
```

The real robot must not move yet. The Ubuntu response should include:

```text
status: PLAN_READY
approval_status: READY_FOR_LOCAL_APPROVAL
local_approval_available: true
request_id: <UUID>
action_sha256: <64-character hash>
planning.result: SUCCESS
```

Copy the `request_id` and `action_sha256`. If the 60-second plan expires while
copying them, request a new plan. Do not try to reuse the expired request.

## 8. Inspect The Frozen Plan On Ubuntu

Ubuntu terminal 3:

```bash
cd ~/franka_ros2_ws

REQUEST_ID='PASTE_REQUEST_ID'
ACTION_SHA256='PASTE_ACTION_SHA256'

python3 scripts/fr3_supervisor_approve.py \
  --request-id "$REQUEST_ID"
```

This displays the status, hash, expiration time, joint delta, planned start,
target, and MoveIt trajectory summary. It does not move the robot.

Check all of these before approving:

```text
Status is READY_FOR_LOCAL_APPROVAL
MoveIt result is SUCCESS
Only fr3_joint3 has a non-zero delta
right.png gives +0.030000 rad
The start and target positions are reasonable
The physical workspace is clear
```

## 9. Approve Exactly One Execution On Ubuntu

While the plan is still valid:

```bash
python3 scripts/fr3_supervisor_approve.py \
  --request-id "$REQUEST_ID" \
  --action-sha256 "$ACTION_SHA256" \
  --execute
```

The tool displays the plan again and asks for an exact confirmation:

```text
EXECUTE <full-request-id>
```

Only after that text matches does the supervisor re-read `/joint_states`,
compare the robot with the frozen start state, require low measured velocity,
and ask MoveIt to execute.

Expected ending:

```text
Final result: VERIFIED SUCCESS
...
"status": "VERIFIED_SUCCESS"
```

The post-execution report must pass direction, motion, target accuracy,
non-commanded drift, and settled velocity.

## 10. Complete The Three-Image Checkpoint

Run one request at a time:

```text
right.png   -> +0.03 rad on fr3_joint3 -> VERIFIED_SUCCESS
left.png    -> -0.03 rad on fr3_joint3 -> VERIFIED_SUCCESS
nothing.png -> NO_ACTION -> Ubuntu is not contacted and the robot does not move
```

Request a new plan for every image. Never execute the same request twice.

## Expected Rejections

```text
INVALID_SIGNATURE    Shared secrets do not match.
REPLAY_REJECTED      The Windows request ID was already submitted.
EXPIRED              The frozen plan exceeded its approval lifetime.
SUPERSEDED           A newer plan replaced this unexecuted plan.
ACTION_HASH_MISMATCH The local hash does not match the frozen action.
START_STATE_CHANGED  The robot moved too far after planning.
ROBOT_NOT_SETTLED    A measured joint velocity is too high before execution.
PLAN_NOT_READY       The plan is expired, consumed, superseded, or already failed.
MOVEIT_*             Planning or execution failed.
VERIFICATION_FAILED  Measured robot motion did not satisfy the verification limits.
```

After any rejection or `FAULT`, inspect the robot and logs. Create a completely
new vision request only after the cause is understood. There is no automatic
retry.

## Stop The Test

1. Stop the Ubuntu supervisor with `Ctrl+C`.
2. Confirm that `/tmp/fr3_ai_supervisor_approval.sock` disappears.
3. Stop the Windows Qwen server with `Ctrl+C`.
4. Stop MoveIt last.

## Next Checkpoint

After the three static-image cases pass repeatedly, follow the plan-only
[bounded static VLA shadow session](static_vla_shadow_session.md). Isaac camera
frames remain the final integration step after the static-image session is
proven.

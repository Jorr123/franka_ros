# Cosmos targets through the existing MoveIt supervisor

The Cosmos adapter now connects to the existing Ubuntu MoveIt planner,
single-use local approval, and measured-motion verifier. It handles **one first
arm target**, not the full 32-step policy chunk. It excludes all gripper commands.

| Interface | Purpose |
| --- | --- |
| `8082 /v1/robot-state` | Existing read-only telemetry used by Windows |
| `8083 /v1/action-proposals/dry-run` | Existing validation-only test; unchanged |
| `8084 /v1/action-proposals/plan` | New bounded Cosmos-to-MoveIt planning request |
| `8084 /v1/action-proposals/status/<request-id>` | Read a stored plan/execution result |
| `/tmp/fr3_cosmos_approval.sock` | Ubuntu-only approval, created only with explicit execution enablement |

HTTP cannot execute or arm a session. The planning endpoint is restricted to
Windows `10.10.10.2` by default and needs no shared-secret configuration. Local
approval uses the existing owner-only Unix socket and typed confirmation.

## Transfer

The new Ubuntu code is under `robot_bridge/` and
`scripts/fr3_cosmos_action_server.py`, together with small reuse hooks in
`scripts/fr3_ai_supervisor.py`. Commit/push the current Franka checkout, then pull
that commit on Ubuntu. These Python changes do not require a ROS rebuild.

The Windows `inference/` changes are in the WSL workspace and remain outside
this Git checkout. Keep the sibling Franka checkout in
`vla_runtime/franka_ros/franka_ros/` for the shared protocol/client imports.

## First run: planning only

Keep the existing robot/MoveIt stack and state server running. On Ubuntu, in a
new terminal with the same ROS environment:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

python3 scripts/fr3_cosmos_action_server.py
```

The new receiver defaults to the actual `/franka/joint_states` topic and domain
42 (an explicitly set domain takes precedence). It reads independent ROS state
using the same strict publisher/freshness checks as the telemetry service. In
this mode the MoveIt execution action client and approval socket are absent.

Keep Cosmos running in WSL. In Windows PowerShell:

```powershell
$runtime = "\\wsl.localhost\Ubuntu-22.04\home\jordan_vlm\vla_runtime"
$python = Join-Path $env:LOCALAPPDATA "CosmosCameraBridge\.venv\Scripts\python.exe"

Invoke-RestMethod "http://10.10.10.1:8084/healthz"

& $python "$runtime\inference\run_cosmos3.py" plan-once `
  --gripper-open-width-m 0.08 `
  --prompt "pick up the object"
```

This captures fresh cameras/state, performs one inference, fetches state again,
and submits the unchanged first absolute arm target. `PLAN_READY` means MoveIt
produced a checked trajectory. `REJECTED` means no executable plan was returned;
the command exits with code 2. Transport/receipt errors exit with code 1, retain
the inputs, and never trigger an automatic retry.

A proposal exceeding the current movement bounds is rejected **before calling
MoveIt**. The adapter does not clip or scale a rejected target or silently
select a different policy row. The operator-requested limits are now 0.1 rad
per joint and 0.4 rad total absolute change for one step.

## What is checked

- The configured 0.1 rad per-joint / 0.4 rad L1 total movement bounds apply to
  the proposal, the actual planning start, and every trajectory waypoint.
- The absolute Cosmos target is retained when MoveIt reads a newer start pose.
  Deltas are recalculated for the backend contract; the target is not rebased.
- The independent ROS state source, freshness, original pose, and raw gripper
  width are checked before planning and again before approved execution.
- Joint limits/margins and collision planning use the existing MoveIt backend
  and its configured planning scene. Scene completeness must be checked on the
  actual robot setup.
- Returned trajectories must contain ordered FR3 joints 1-7, finite values,
  increasing timestamps, and endpoints within the goal tolerance. Oversized
  review payloads are rejected (2048 waypoints / approximately 2 MiB maximum).
- Initial velocity/acceleration scaling is 0.03. Start-state, goal and measured
  target tolerances are at most 0.002 rad in this adapter. Joint 7 uses the same
  measured target tolerance. Existing settled-velocity and minimum-motion
  checks remain enabled.
- Initial movement/tolerance ceilings cannot be raised through this launcher's
  arguments. Lower limits can be selected. Legacy tools keep their own defaults.

The mapping supplies numeric DROID joint targets directly to FR3 joints 1-7.
It does not establish Panda/FR3 kinematic equivalence or task quality. Check the
joint conventions and actual planned movement before enabling policy execution.
The legacy action schema requires a confidence value; the adapter supplies 1.0
as an interface constant, not a Cosmos-provided confidence estimate.

## Enable one locally approved movement

After reviewing the planning behavior, stop only the new port-8084 process and
restart it on Ubuntu with explicit execution enablement:

```bash
python3 scripts/fr3_cosmos_action_server.py --enable-local-execution
```

This creates the approval socket; it does not move the robot. Generate a **new**
`plan-once` request from Windows. The printed request UUID and action SHA256
identify the exact frozen plan. Inspect it on Ubuntu:

```bash
python3 scripts/fr3_supervisor_approve.py \
  --socket /tmp/fr3_cosmos_approval.sock \
  --request-id REQUEST_UUID
```

For a reviewed plan with a clear workspace and the stop available, explicitly
request one execution:

```bash
python3 scripts/fr3_supervisor_approve.py \
  --socket /tmp/fr3_cosmos_approval.sock \
  --request-id REQUEST_UUID \
  --action-sha256 ACTION_SHA256 \
  --execute
```

The existing approval tool displays the plan and requires the exact text
`EXECUTE REQUEST_UUID`. A successful request consumes the frozen plan before
trajectory submission. The existing backend checks start pose/velocity again,
executes that trajectory, waits for measured verification, and returns
`VERIFIED_SUCCESS` or a terminal failure. A failed/expired/consumed/superseded
plan cannot be retried as another execution. Another movement requires another
inference, plan and explicit approval.

The original observation has a **30-second total lifetime**, including capture,
inference, planning and review. Approval expiry is capped by that deadline; a
17-second inference can leave less than 13 seconds for planning and review.
Do not rush approval. If review cannot be completed, allow expiry and obtain a
new inference. `--plan-ttl` can shorten the lifetime; it cannot extend the
original observation deadline.

## Get the result on Windows

For a plan created while local execution was enabled:

```powershell
& $python "$runtime\inference\run_cosmos3.py" plan-status `
  --request-id REQUEST_UUID `
  --output-file "$env:TEMP\cosmos_execution_result.json"
```

This is a read-only GET. The result can report `READY_FOR_LOCAL_APPROVAL`,
`VERIFIED_SUCCESS`, `FAULT`, `EXPIRED`, or `SUPERSEDED`. The `result.verification`
field includes the backend's measured joint-position and settled-velocity
report after success. A timeout or unavailable result is not a reason to send
another execution request.

Windows preserves the normal inference artifacts plus `plan_proposal.json`,
`plan_receipt.json` (including the full checked trajectory), and
`plan_result.json`. Ubuntu records planning receipts and local execution events
in `log/cosmos_action_plans.jsonl`. A failed planning audit write revokes local
approval. Failure to write the pre-execution event blocks execution; a write
failure after execution is reported as an audit error with the actual result.
All frozen plans/results are in memory and disappear when the receiver stops.

## Offline verification

Tests use fake MoveIt, simulated measurements, localhost HTTP and Unix sockets;
they do not command a physical robot:

```bash
python3 -m unittest tests.test_cosmos_action_planning \
  tests.test_fr3_action_protocol tests.test_fr3_supervisor_approval \
  tests.test_robot_bridge_dry_run -v
```

Windows tests:

```powershell
& $python -m unittest discover -s "$runtime\inference" -t "$runtime" -p "test_*.py" -v
```

Native Ubuntu/MoveIt integration and the first physical Cosmos-derived movement
still need to be checked on the actual robot computer after transfer.

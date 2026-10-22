# One-command Cosmos movement

This opt-in mode removes plan-ID copying and typed approval. One Windows
`execute-once` command runs a fresh inference, submits the first arm target,
plans through MoveIt, executes one bounded trajectory, and waits for measured
verification. It can move the real arm. Gripper commands remain excluded.

The default server still only plans. `plan-once` never executes, even on an
automatically armed server. Automatic execution has a separate request mode,
`EXECUTE_ONCE`, and path, `/v1/action-proposals/execute-once`, on port 8084.

## Ubuntu

Pull the new commit. Keep Robot + Read State running. Stop only the port-8084
Cosmos planning server with Ctrl+C, then in that same ROS-configured terminal:

```bash
cd ~/franka_ros2_ws
git pull --ff-only
export ROS_DOMAIN_ID=42
python3 scripts/fr3_cosmos_action_server.py --allow-one-auto-execution
```

If opening a new terminal, source `/opt/ros/humble/setup.bash` and the workspace's
`install/setup.bash` first. No secret or approval command is required. The
existing `--enable-local-execution` mode remains available separately; the two
startup modes cannot be selected together.

Starting the server does not move the robot. Its health endpoint reports
`auto_execution_armed: True`. Default access remains restricted to Windows
`10.10.10.2` on the named control-network interface.

## Windows PowerShell

With Cosmos running, the robot stationary, a clear workspace, and the stop
within reach:

```powershell
$runtime = "\\wsl.localhost\Ubuntu-22.04\home\jordan_vlm\vla_runtime"
$python = Join-Path $env:LOCALAPPDATA "CosmosCameraBridge\.venv\Scripts\python.exe"

& $python "$runtime\inference\run_cosmos3.py" execute-once `
  --gripper-open-width-m 0.08 `
  --prompt "pick up the object"
```

There is no confirmation prompt. This command requests real movement. The
existing checks still apply: 0.1 rad maximum per joint, 0.4 rad absolute sum,
fresh/source-consistent measurements, starting-pose drift, joint limits,
MoveIt collision planning and waypoint bounds, settled-state checks, original
30-second observation deadline before execution, and measured verification.
Trajectory speed/acceleration and verification tolerances are unchanged.
The first target remains unchanged, with no clipping or scaling.

Successful output ends with `VERIFIED_SUCCESS`. The receipt includes
`execution.verification.status: PASSED` and measured joint/velocity details.
This is one arm step, not a complete pickup task or a continuous policy loop.
DROID-to-FR3 joint conventions and task suitability still need physical review.

## One attempt, then disarm

The server consumes its automatic permission before requesting execution.
Success, failed verification, stale-state rejection at execution, or a lost
execution outcome cannot rearm it. A second request returns
`AUTO_EXECUTION_DISARMED`; simultaneous requests cannot produce two movements.
Planning/bounds rejections before an execution attempt leave it armed.

The default is one automatic attempt per server lifetime. To deliberately
permit another attempt, stop and restart with the same startup flag. There is
no network rearm endpoint and no automatic retry or loop.

If an error, timeout or interrupted connection occurs, do not assume the robot
stayed still. The outcome may be unknown. Read the stored request's result
using `plan-status` and its ID from `execution_proposal.json`, and inspect the
robot/Ubuntu log before deliberately rearming:

```powershell
& $python "$runtime\inference\run_cosmos3.py" plan-status --request-id REQUEST_UUID
```

Windows saves `execution_proposal.json`, `execution_receipt.json` when received,
and `execution_result.json`, plus the normal raw observations/predictions.
Unknown outcomes are recorded with `robot_execution: null`. A receipt/audit
failure after movement preserves the actual execution result and must not be
interpreted as no movement. Ubuntu retains the existing JSONL audit log.

## Tests

The automatic execution, concurrency, disarming, default-off and fault paths
are tested with fake MoveIt and local HTTP sockets. Windows tests use simulated
cameras and model outputs. They do not send physical robot commands:

```bash
python3 -m unittest tests.test_cosmos_execute_once tests.test_cosmos_action_planning \
  tests.test_fr3_action_protocol tests.test_fr3_supervisor_approval \
  tests.test_robot_bridge_dry_run -q
```

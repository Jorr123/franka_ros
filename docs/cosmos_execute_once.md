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
export ROS_DOMAIN_ID=99
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

## Temporary report-only accuracy checks

Normal enforcement is the default (`--verification-mode enforce`). To record
small accuracy failures without faulting the completed controller execution,
start Ubuntu with:

```bash
python3 scripts/fr3_cosmos_action_server.py --allow-one-auto-execution \
  --verification-mode report-only
```

The Windows `execute-once` command is unchanged. Failed target accuracy,
direction or minimum-motion checks return `EXECUTED_WITH_WARNINGS`, not
`VERIFIED_SUCCESS`. The receipt and audit retain the failed report, measured
positions and velocities. Normal enforcement failures now retain these details
too. A passing report remains `VERIFIED_SUCCESS` in either mode.

Controller faults, missing measurements, excessive settled velocity,
non-commanded drift, or any final target error greater than the configured
per-joint movement limit (normally 0.1 rad) still stop execution. Existing
pre-execution and planning checks remain enforced. Report-only mode still
disarms after one attempt; it does not enable a loop or gripper commands.
Stop and restart without the option to restore normal enforcement.

## Allow several separate requests

To avoid restarting between successful steps, choose a finite budget at startup:

```bash
python3 scripts/fr3_cosmos_action_server.py --allow-auto-executions 5 \
  --verification-mode report-only
```

Use this instead of `--allow-one-auto-execution`; the flags are mutually
exclusive. Each Windows `execute-once` command still captures fresh inputs,
infers, and requests just one movement. Up to five execution attempts are
available for this server lifetime. Successful execution or a report-only
warning permits the next request after its receipt is saved. The receipt and
`/healthz` expose `auto_executions_remaining`.

The budget is consumed before execution. Controller/verification faults,
unknown outcomes, execution guard failures or receipt audit failures discard
the remaining budget and disarm the server. Planning/bounds rejections before
an attempt do not consume the budget. No automatic retry, task loop, gripper
control or network rearm is introduced. Omitting both startup flags still
keeps execution disabled.

## Configurable action chunks

Windows can select `--actions-per-chunk K` (`1..32`, default `1`) on
`plan-once`, `execute-once` and `execute-loop`. On the loop,
`--num-chunks N` is an alias for `--max-steps N`: N counts separate fresh
inference/execution cycles, and K counts consecutive policy actions inside
each trajectory. For example:

```powershell
& $python "$runtime\inference\run_cosmos3.py" execute-loop `
  --gripper-open-width-m 0.08 --prompt "pick up the object" `
  --actions-per-chunk 4 --num-chunks 5
```

Start this Ubuntu receiver with `--allow-auto-executions 5` for that example.
One chunk consumes one attempt. The default nominal action period is 1/15 s,
configurable via `--action-period-seconds`. Ubuntu stretches the complete
prefix to respect the checked-in joint limits scaled by the configured
velocity/acceleration factors (currently 0.2). Receipt `plan.chunk` records
the nominal/effective periods, time scale, and waypoint sample indices.

For K > 1, the adapter constructs one C2 quintic trajectory through all
unchanged targets. Adjacent movement in the same direction can flow through
interior targets without stopping. Turns may require zero velocity. The
trajectory starts and ends with zero velocity/acceleration. All intermediate
targets and dense interpolated samples must remain within 0.1 rad per joint
and 0.4 rad total from the original chunk start; larger K does not expand
those bounds. The hash covers all selected targets and nominal timing.

Dense collision/bounds samples are checked through MoveIt's
`/check_state_validity` service at planning and again before execution.
Sampling bounds are 0.001 rad per joint and 0.05 seconds, with at most 2048
samples. Checks depend on the current planning scene; the adapter rejects
blocked paths rather than using OMPL to change intermediate targets. Missing
services, stale observations, invalid limits, oversized paths and trajectories
exceeding the execution timeout are rejected. Gripper commands remain held.
Final measured verification is at the end of the trajectory. New observations
are acquired between chunks, with no mid-motion replacement.

ROS 2 messages/services and PyYAML are required on Ubuntu. Chunk geometry,
timing, bounds, collision failure and service calls are tested with fake ROS
types and responses; hardware execution of this adapter still needs testing.
Use `plan-once --actions-per-chunk 4 ...` to review the same path without
movement. K=1 retains the existing single-target planning behavior.

## One attempt, then disarm

With `--allow-one-auto-execution`, the server consumes its automatic permission before requesting execution.
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

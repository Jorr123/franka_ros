# Windows Codex Handoff: Bounded Two-Joint FR3 Sessions

> Implementation status: completed in this workspace using protocol schema
> version 2. Use `docs/bounded_multi_joint_session.md` for the current commands.
> The sections below preserve the original implementation requirements.

## Purpose

Prepare the Windows bridge for bounded two-joint relative-joint actions while
GR00T is being installed. This is a protocol/client change, not a model change.

Do not control the real robot during development. The updated Windows client
must refuse to submit a two-joint action until Ubuntu advertises matching
multi-joint bounded-session support.

## Current Proven State

The real FR3 has successfully executed this action locally on Ubuntu through
MoveIt, `ros2_control`, and measured post-execution verification:

```json
{
  "action": "joint_deltas",
  "deltas": {
    "fr3_joint1": 0.05,
    "fr3_joint3": -0.05
  },
  "confidence": 1.0
}
```

Observed result:

```text
Planning result: SUCCESS
Execution result: SUCCESS
fr3_joint1: direction PASS, motion PASS, accuracy PASS
fr3_joint3: direction PASS, motion PASS, accuracy PASS
Non-commanded drift: PASS
Settled velocity: PASS
Final result: VERIFIED SUCCESS
```

This proves the Ubuntu MoveIt execution script can handle two joints. It does
not authorize a remote model to use `0.05 rad` actions. The initial remote
limits must remain `0.01 rad` per joint and `0.02 rad` total per step.

Before this change, the bounded remote protocol was intentionally
one-joint-only. Its fields and checks included:

```text
allowed_joint
--joint
--max-moving-joints 1
verification must contain exactly one commanded joint
```

## Target Session Limits

Use this strict session representation:

```json
{
  "allowed_joints": ["fr3_joint1", "fr3_joint3"],
  "max_moving_joints": 2,
  "max_delta": 0.01,
  "max_total_delta": 0.02,
  "max_steps": 2,
  "max_duration_seconds": 60.0,
  "max_cumulative_delta": 0.04
}
```

Definitions:

- `max_delta` is the maximum absolute delta for each individual joint.
- `max_total_delta` is the L1 sum `sum(abs(delta))` for one action.
- `max_cumulative_delta` is the sum of the verified per-step L1 totals.
- Only successfully verified steps consume the cumulative allowance.
- An action may use a non-empty subset of `allowed_joints`, but it may not use
  more than `max_moving_joints`.

## Required Windows Changes

Work in `C:\vlm_bridge`. Inspect the current files before editing and preserve
unrelated local changes.

### 1. `fr3_action_protocol.py`

Create a coordinated multi-joint schema version. Prefer `SCHEMA_VERSION = 2`
because the session-arm limits are changing incompatibly.

Change the session-arm limits from:

```text
allowed_joint
```

to exactly:

```text
allowed_joints
max_moving_joints
max_delta
max_total_delta
max_steps
max_duration_seconds
max_cumulative_delta
```

Validation requirements:

- `allowed_joints` must be a non-empty list of unique valid FR3 joint names.
- Store or transmit the joint names in canonical FR3 order.
- Reject duplicates, unknown joints, booleans, empty strings, and extra keys.
- `max_moving_joints` must be an integer in
  `[1, len(allowed_joints)]`.
- All numeric limits must be finite and positive.
- `max_total_delta <= max_delta * max_moving_joints`.
- `max_cumulative_delta >= max_total_delta`.
- Preserve canonical JSON, HMAC-SHA256 signatures, UUIDv4 request IDs,
  freshness checks, action hashes, and exact-key validation.
- Do not silently reinterpret a schema-v1 one-joint arm envelope as schema v2.

Keep the session-step action format unchanged:

```json
{
  "action": "joint_deltas",
  "deltas": {
    "fr3_joint1": 0.01,
    "fr3_joint3": -0.01
  },
  "confidence": 1.0
}
```

### 2. `fr3_remote_session_control.py`

Make `--joint` repeatable for `arm`:

```powershell
python fr3_remote_session_control.py arm `
  --joint fr3_joint1 `
  --joint fr3_joint3 `
  --max-moving-joints 2 `
  --max-delta 0.01 `
  --max-total-delta 0.02 `
  --max-steps 2 `
  --max-cumulative-delta 0.04
```

Add `--max-moving-joints` and `--max-total-delta`. Require at least one
explicit `--joint` for schema-v2 arming. Normalize repeated joints into a
unique canonical list and reject duplicates rather than silently removing
them.

Update status output to print:

```text
Allowed joints
Maximum moving joints
Per-joint delta limit
Per-step total-delta limit
Cumulative verified-delta usage
```

Remote arming creates authority only. It must not move the robot.

### 3. Add `bounded_joint_action_session.py`

Add a small model-independent client for testing action files. Do not modify
Qwen or add GR00T in this change.

Suggested CLI:

```powershell
python bounded_joint_action_session.py `
  --action-json-file C:\vlm_bridge\actions\two_joint_forward.json `
  --action-json-file C:\vlm_bridge\actions\two_joint_reverse.json `
  --max-moving-joints 2 `
  --max-delta 0.01 `
  --max-total-delta 0.02 `
  --max-session-seconds 50 `
  --log-file C:\vlm_bridge\session_logs\two_joint_dry.jsonl
```

Default behavior must be local dry-run only:

```text
Robot execution: False
Ubuntu contacted: False
```

Require the explicit flag below before any bounded session step can be sent:

```text
--run-armed-session
```

For each action file:

1. Parse exactly one JSON object.
2. Require `action == "joint_deltas"`.
3. Validate confidence and finite numeric deltas.
4. Remove only exact zero values when computing the moving set.
5. Require a non-empty moving set.
6. Require the moving set to be a subset of the armed `allowed_joints`.
7. Enforce `len(moving) <= max_moving_joints`.
8. Enforce every `abs(delta) <= max_delta`.
9. Enforce `sum(abs(delta)) <= max_total_delta`.
10. Enforce the cumulative verified L1 limit.
11. Submit at most one request at a time.
12. Never retry automatically.
13. Advance the sequence and cumulative value only after a valid
    `VERIFIED_SUCCESS` receipt.
14. On rejection, timeout, malformed receipt, verification failure, or stop
    request, terminate the session and do not send another action.

Use the SHA-256 of the action file bytes for the envelope evidence digest and
identify the source as a manual Windows action-file test. Do not pretend the
file is a camera image.

### 4. Health and Capability Checks

Before `--run-armed-session` can send anything, require Ubuntu `/health` to
advertise all of the following:

```text
mode == BOUNDED_SESSION_EXECUTION
remote_direct_execution_available == false
session_step_available == true
multi_joint_bounded_sessions == true
session schema version == 2
one fresh ARMED session exists
```

The Windows requested limits must be equal to or stricter than the armed Ubuntu
limits. Refuse execution if `allowed_joints`, action semantics, schema version,
or cumulative accounting are missing or ambiguous.

Windows must fail closed while Ubuntu still reports the old one-joint schema.
Do not add a compatibility fallback that weakens validation.

### 5. Receipt Validation

For each verified step, require:

- HTTP success and `status == "VERIFIED_SUCCESS"`.
- Exact session ID, sequence, request ID, and action SHA-256.
- `verification.status == "PASSED"`.
- The receipt's commanded-joint set exactly equals the nonzero action-joint
  set. No missing and no extra commanded joints.
- Every commanded joint has `direction_ok`, `movement_ok`, and `target_ok` set
  to true.
- `non_commanded_drift_ok == true`.
- `settled_velocity_ok == true`.
- Reported requested deltas match the submitted deltas.
- Reported session cumulative usage equals the previous verified cumulative
  value plus the current action's L1 total.

## Focused Tests

Add or update tests for:

1. A valid two-joint schema-v2 arm envelope.
2. Empty, duplicate, unknown, or more-than-two allowed joints.
3. Invalid `max_moving_joints`.
4. Per-joint delta rejection.
5. Per-step L1 total-delta rejection.
6. Cumulative L1 rejection.
7. An action using a joint outside the allowlist.
8. A valid one-joint subset inside a two-joint session.
9. A valid two-joint action.
10. Receipt missing one commanded joint.
11. Receipt containing an extra commanded joint.
12. Receipt with one failed direction, motion, or accuracy flag.
13. Default dry-run performs no HTTP request.
14. Schema-v1 or capability-missing Ubuntu health is rejected.
15. Signature, timestamp, replay ID, and action hash checks remain intact.

Run the Windows tests without contacting Ubuntu or the robot.

## Manual Dry-Run Files

Create these only after the code and tests pass.

`C:\vlm_bridge\actions\two_joint_forward.json`:

```json
{
  "action": "joint_deltas",
  "deltas": {
    "fr3_joint1": 0.01,
    "fr3_joint3": -0.01
  },
  "confidence": 1.0
}
```

`C:\vlm_bridge\actions\two_joint_reverse.json`:

```json
{
  "action": "joint_deltas",
  "deltas": {
    "fr3_joint1": -0.01,
    "fr3_joint3": 0.01
  },
  "confidence": 1.0
}
```

Expected dry-run ending:

```text
Actions validated: 2
Maximum moving joints: 2
Maximum per-joint delta: 0.010000 rad
Maximum step total: 0.020000 rad
Cumulative proposed delta: 0.040000 rad
Ubuntu contacted: False
Robot execution: False
```

## Non-Goals

Do not do any of the following in this Windows change:

- Do not enable the real robot.
- Do not call ROS, MoveIt, `ros2_control`, or `libfranka` from Windows.
- Do not allow all seven joints.
- Do not use `0.05 rad` as a model action limit.
- Do not change the Qwen prompt or Qwen server.
- Do not integrate GR00T yet.
- Do not add retries, fallback planners, correction motions, or automatic
  re-arming.
- Do not remove signatures, freshness checks, request IDs, sequence numbers,
  action hashes, bounded duration, or cumulative limits.
- Do not claim Windows integration is complete until the matching Ubuntu
  schema-v2 supervisor is implemented and tested.

## Integration Sequence After Windows Work

1. Finish Windows code and offline tests.
2. Keep `--run-armed-session` unused.
3. Implement the matching Ubuntu schema-v2 supervisor separately.
4. Run cross-machine health and dry-run validation without an armed session.
5. Run a two-action Ubuntu MoveIt plan-only test.
6. Arm a two-step real session with `0.01 rad` per joint only.
7. Require both steps to return `VERIFIED_SUCCESS`.
8. Only then connect the GR00T relative-joint adapter in shadow mode.

## Completion Report Requested From Windows Codex

Return:

- Files changed.
- Exact schema-v2 JSON examples.
- Tests added and their results.
- Dry-run command and output.
- Confirmation that no request reached Ubuntu and no robot execution occurred.
- Any required matching Ubuntu changes, without implementing or pretending
  they already exist on Windows.

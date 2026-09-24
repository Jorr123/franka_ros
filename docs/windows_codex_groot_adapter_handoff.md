# Windows Codex Handoff: GR00T To FR3 Action Adapter

Copy everything inside the **Prompt For Windows Codex** section into Codex on
the Windows computer where GR00T is installed.

## Current Project Goal

The final goal is to deploy a GR00T policy on Windows and safely execute its
actions on a real Franka Research 3 robot connected to an Ubuntu mini PC.

The intended architecture is:

```text
Windows
GR00T policy
  -> inspected raw policy action
  -> GR00T-to-FR3 adapter
  -> bounded named relative-joint action
  -> authenticated HTTP session request

Ubuntu
strict safety validation
  -> MoveIt planning
  -> ros2_control
  -> franka_hardware / libfranka
  -> real FR3
  -> measured post-execution verification
```

Windows must never call `libfranka`, `ros2_control`, or the robot directly.

## Proven Progress

### Robot and ROS

- Ubuntu runs ROS 2 Humble, MoveIt, `ros2_control`, and `franka_ros2`.
- The real FR3 is at `172.16.0.2`.
- Real robot joint states are available on `/joint_states`.
- MoveIt joint-goal planning and execution work on the real robot.
- Post-execution verification reads measured joint positions and velocities.
- Real-to-simulation joint-state mirroring into Isaac Sim has previously
  worked.

### Windows and Ubuntu Network

```text
Windows: 10.10.10.2
Ubuntu:  10.10.10.1
FR3:     172.16.0.2
```

- Windows and Ubuntu communicate over authenticated HTTP.
- The robot network remains separate from the Windows-to-Ubuntu network.
- HMAC-SHA256 signatures, UUID request IDs, freshness checks, replay
  protection, action hashes, sequence numbers, deadlines, and bounded sessions
  are already implemented.
- There is no generic direct remote execution endpoint.

### Qwen and Earlier Model Tests

- Qwen2.5-VL runs on the Windows GPU.
- Qwen has successfully classified saved red-marker images as left/right.
- Those observations were converted to small joint actions and executed
  through MoveIt.
- This proved the communication and model-to-robot structure, but the Qwen
  action mapping was intentionally simple and partly deterministic.

### Proven Relative-Joint Execution

The Ubuntu execution code has successfully moved two joints together:

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

The initial remote/model limit remains much smaller: `0.01 rad` per joint.

### Proven Schema-V2 Two-Joint Remote Session

The latest real-robot test ran from Windows using two saved action files:

Step 1:

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

Step 2:

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

Windows reported:

```text
Status:                    SESSION_COMPLETED
Actions validated:         2
Maximum moving joints:     2
Maximum per-joint delta:   0.010000 rad
Maximum step total:        0.020000 rad
Cumulative proposed/done:  0.040000 rad
Ubuntu contacted:          True
Robot execution:           True
```

Ubuntu measured verification reported `VERIFIED SUCCESS` for both steps.
Both commanded joints passed direction, motion, and target checks. Uncommanded
drift and settled velocity also passed.

Approximate net residual after the forward/reverse pair:

```text
fr3_joint1: +0.00035 rad
fr3_joint3: +0.00027 rad
```

This proves the multi-joint transport and execution pipeline. It does not yet
prove GR00T integration because the actions came from static JSON files.

## Current Protocol Contract

The session protocol is schema version 2. A session defines:

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

- `max_delta`: maximum absolute delta for each individual joint.
- `max_total_delta`: per-action L1 sum, `sum(abs(delta))`.
- `max_cumulative_delta`: sum of verified per-action L1 totals.
- Only measured `VERIFIED_SUCCESS` steps advance the sequence and cumulative
  value.
- One request may be in flight.
- There are no automatic retries.
- Any ambiguous result terminates the session.

The action object currently accepted by the Windows and Ubuntu validators is:

```json
{
  "action": "joint_deltas",
  "deltas": {
    "fr3_joint1": 0.008,
    "fr3_joint3": -0.006
  },
  "confidence": 1.0
}
```

Joint names are always explicit. Raw array order must never be trusted without
documented model metadata.

## Relevant Windows Files

Expected directory:

```text
C:\vlm_bridge
```

Important existing files:

```text
fr3_action_protocol.py
fr3_remote_session_control.py
bounded_joint_action_session.py
static_vla_execution_session.py
closed_loop_vla_execution_session.py
qwen_vlm_server.py
```

The action-file client is dry-run-only unless the exact flag below is present:

```text
--run-armed-session
```

Do not use that flag during GR00T adapter development.

## Immediate Goal

Inspect the installed GR00T model and create a model-independent adapter that
can convert one documented GR00T action into the existing named FR3
`joint_deltas` object. The entire first phase must run offline on Windows.

Do not assume GR00T emits seven relative FR3 joint positions. Determine the
actual semantics from the installed checkpoint, embodiment configuration,
processor, normalization metadata, and inference API.

The first completion checkpoint is:

```text
saved/test observation
  -> GR00T inference on Windows
  -> raw action inspection
  -> documented conversion
  -> named joint-delta JSON file
  -> bounded_joint_action_session.py dry-run
  -> Ubuntu contacted: False
  -> Robot execution: False
```

## Prompt For Windows Codex

```text
I am integrating an NVIDIA GR00T policy running on this Windows computer with
an existing safety-controlled Franka Research 3 pipeline. Please inspect the
local GR00T installation and implement only the first offline adapter stage.

Do not contact Ubuntu, ROS, MoveIt, Isaac Sim, or the real robot. Do not use
--run-armed-session. Do not modify or weaken the existing HMAC protocol,
bounded-session limits, action validator, or safety checks.

CURRENT PROVEN PIPELINE

Windows already has C:\vlm_bridge with:

- fr3_action_protocol.py
- fr3_remote_session_control.py
- bounded_joint_action_session.py
- static_vla_execution_session.py
- closed_loop_vla_execution_session.py

Ubuntu already safely accepts schema-v2 bounded named relative-joint actions,
plans through MoveIt, executes through ros2_control/libfranka, and verifies the
real measured joint states. A two-step real test using fr3_joint1 and
fr3_joint3 completed successfully.

The current action contract is exactly:

{
  "action": "joint_deltas",
  "deltas": {
    "fr3_joint1": 0.008,
    "fr3_joint3": -0.006
  },
  "confidence": 1.0
}

Initial model-side limits are:

- allowed joints: fr3_joint1 and fr3_joint3 only
- maximum moving joints: 2
- maximum absolute delta per joint: 0.01 rad
- maximum L1 total per action: 0.02 rad
- maximum two-step cumulative L1 total: 0.04 rad

IMMEDIATE TASK

1. Inspect the local GR00T repository, installed package, checkpoint, model
   configuration, embodiment configuration, processor, inference entry point,
   and normalization metadata before editing anything.

2. Report the exact GR00T setup you find:

   - repository/package version and checkpoint/model ID;
   - required observation keys;
   - whether images, language, and robot state are required;
   - raw action tensor shape and dtype;
   - whether one inference returns one action or an action chunk;
   - exact action dimensions and ordering;
   - whether actions are joint-space or Cartesian;
   - whether actions are relative or absolute;
   - units before and after decoding;
   - normalization/de-normalization method and metadata source;
   - gripper dimension semantics, if present.

3. Do not guess missing semantics. If the installed checkpoint does not expose
   enough metadata to map its actions safely, stop after creating an
   inspect-only report/tool and explain exactly what checkpoint or embodiment
   information is missing.

4. Add a new script named:

   C:\vlm_bridge\groot_joint_action_adapter.py

   It must be dry-run/inspect-only by default and have no HTTP, ROS, MoveIt,
   libfranka, or robot execution interface.

5. Separate the implementation into two explicit stages:

   A. Raw inspection:
      - run one inference using a saved observation or an existing dataset
        sample compatible with the installed checkpoint;
      - print and save the raw action shape, dtype, finite-value status, raw
        values, and any action-chunk dimensions;
      - save the model/config/normalization metadata used for interpretation.

   B. Conversion, enabled only when the semantics are confirmed:
      - decode/de-normalize using the checkpoint's documented metadata;
      - map dimensions to explicit FR3 joint names;
      - never rely on an undocumented array order;
      - never silently drop nonzero dimensions;
      - never silently clip an unsafe value into range;
      - reject NaN, infinity, shape mismatch, missing metadata, unknown joints,
        absolute values presented as deltas, unsupported Cartesian actions, and
        ambiguous action chunks;
      - for an action chunk, do not execute or export the whole chunk. Inspect
        its temporal semantics first and export at most one explicitly selected
        next action only after those semantics are confirmed;
      - write exactly one action JSON object compatible with
        bounded_joint_action_session.py.

6. The existing schema requires a confidence field. If GR00T does not provide
   calibrated confidence, do not pretend that it does. For this offline test,
   a fixed confidence may be included only if it is clearly named and
   documented as adapter/test validity rather than model uncertainty.

7. Add focused offline tests in:

   C:\vlm_bridge\tests\test_groot_joint_action_adapter.py

   Cover at least:

   - correct documented dimension-to-joint mapping;
   - deterministic de-normalization using known fixture metadata;
   - wrong tensor dimension;
   - NaN and infinity;
   - unknown or duplicate joint mapping;
   - missing normalization metadata;
   - unsupported absolute-joint or Cartesian action semantics;
   - ambiguous action chunk rejection;
   - per-joint value above 0.01 rad;
   - more than two moving joints;
   - L1 total above 0.02 rad;
   - a valid one-joint subset;
   - a valid fr3_joint1/fr3_joint3 action;
   - output JSON accepted by bounded_joint_action_session.py dry-run;
   - proof that no HTTP request is made and no robot-execution function exists.

8. Preserve existing local changes. Keep modifications focused on the new
   adapter, its tests, and a short GR00T adapter README. Do not refactor the
   working safety bridge.

9. Do not install or upgrade packages until you inspect the active environment
   and report the current Python, CUDA, PyTorch, Transformers, GR00T, and other
   relevant versions. Avoid changing the environment that already runs the
   model unless a specific missing dependency is proven necessary.

10. After implementation, run only offline tests and one offline adapter dry
    run. The required final result is:

    - GR00T raw output semantics documented from local evidence;
    - one converted named joint-delta JSON file, or a clear fail-closed report
      explaining why conversion is not yet justified;
    - all adapter tests passing;
    - Ubuntu contacted: False;
    - Robot execution: False.

Before changing code, show me what you found about the installed GR00T action
space and the small implementation plan. Do not move the robot.
```

## What Comes After This Checkpoint

Only after the adapter's semantics and offline tests are proven:

1. Run several GR00T observations through the adapter in shadow mode.
2. Inspect the distribution, signs, scale, saturation, and action chunks.
3. Submit one GR00T-generated action to Ubuntu plan-only, without execution.
4. Arm one real step limited to `0.01 rad` per joint.
5. Require measured `VERIFIED_SUCCESS` before trying a second step.
6. Add a camera or Isaac observation source only after model action conversion
   is trusted.

Do not expand to all seven joints or continuous control during the first
adapter checkpoint.

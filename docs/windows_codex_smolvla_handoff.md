# Windows Codex Handoff: SmolVLA To FR3

Date: August 19, 2026

## Purpose

Continue the FR3 VLA project on the Windows computer with the NVIDIA GPU. The
immediate objective is to evaluate raw SmolVLA inference and define a future
FR3 fine-tuning dataset. Do not command Ubuntu or the real robot during this
first stage.

The Ubuntu mini PC remains the only computer allowed to plan and execute real
FR3 motion:

```text
Windows camera + policy
  -> model-specific action adapter
  -> authenticated bounded proposal
  -> Ubuntu safety supervisor
  -> MoveIt
  -> ros2_control
  -> franka_hardware / libfranka
  -> real FR3
  -> measured post-execution verification
```

Windows must never call `libfranka`, the hardware command interfaces, or the
robot directly.

## Computers And Network

```text
Windows policy PC:         10.10.10.2/24
Ubuntu Windows interface:  10.10.10.1/24
Ubuntu robot interface:    172.16.0.x/24
FR3:                       172.16.0.2
```

Keep the Windows-Ubuntu and Ubuntu-FR3 networks separate.

## Proven Project State

- Ubuntu runs ROS 2 Humble, MoveIt, `ros2_control`, `franka_hardware`, and
  `libfranka`.
- MoveIt has planned and executed measured relative-joint goals on the real
  FR3.
- Positive and negative one-joint commands have passed measured verification.
- Two-joint commands have passed locally and through the authenticated Windows
  bounded-session protocol.
- The proven remote action schema uses explicit joint names, per-step limits,
  cumulative limits, one request in flight, no retries, and terminate-on-fault.
- Qwen on Windows has previously produced constrained visual decisions that
  were converted to bounded actions and executed through Ubuntu.
- Real FR3 joint states have previously driven an Isaac Sim FR3 model.
- A discrete Ubuntu keyboard jogger now supports `--auto-execute`; each fresh
  arrow key plans, executes, verifies, and only then accepts another key.

The newest keyboard feature is documented in:

```text
docs/discrete_keyboard_joint_jog.md
```

## Current Action Contract

The tested model-independent proposal is:

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

The safety-relevant limits are owned by Ubuntu. A model output must never be
trusted merely because it has seven values. Joint order, units, normalization,
absolute-versus-relative meaning, control period, and action horizon must be
documented by the model adapter.

The current protocol still has a `confidence` field because it predates the
true VLA work. SmolVLA and GR00T do not provide a calibrated safety confidence.
Do not invent or use model confidence as a hardware safety signal. Preserve the
existing protocol until a deliberate schema migration is designed and tested.

## Why SmolVLA Is Being Evaluated

SmolVLA is being considered as the first engineering baseline because it is
smaller than GR00T, can be configured around one real camera, and uses the
LeRobot dataset ecosystem. GR00T remains installed as a later comparison.

Raw SmolVLA is suitable only for an offline software smoke test. It is not a
drop-in FR3 controller. A real deployment requires fine-tuning with the exact
FR3 observation and action representation.

Proposed initial fine-tuning schema:

```text
observation.images.front: RGB image from one fixed camera
observation.state:        seven measured FR3 joint positions
task:                     language instruction
action:                   seven explicitly ordered relative joint deltas
```

Gripper state/action should be added only when the gripper is intentionally
part of the task and dataset.

## Immediate Windows Task

Work offline only:

1. Create a separate SmolVLA/LeRobot environment. Do not modify the working
   Qwen or GR00T environment.
2. Record the exact LeRobot, SmolVLA, PyTorch, CUDA, Python, and Transformers
   versions.
3. Load the official raw SmolVLA checkpoint.
4. Run one inference call using a saved image, dummy seven-joint state, and a
   language instruction.
5. Print the raw action tensor shape, action chunk length, finite-value check,
   minimum/maximum values, inference time, and GPU memory.
6. Save the unmodified raw output and all model/config metadata to files.
7. Do not map the output to FR3 joints yet.
8. Do not contact `10.10.10.1`, ROS, MoveIt, or the real robot.

Completion checkpoint:

```text
SmolVLA loads successfully
one image/state/instruction produces a finite action chunk
input feature names and output semantics are documented
Ubuntu contacted: False
Robot execution: False
```

## Relevant Copied Code

The transfer bundle contains:

```text
windows_vlm_bridge/   Windows protocol, Qwen, and bounded-session clients
scripts/              Ubuntu supervisors, MoveIt clients, validators, jogger
tests/                Focused software tests
docs/                 Architecture, safety, progress, and runbooks
```

Do not copy Ubuntu `build/`, `install/`, or `log/` to Windows. They are Linux
ROS artifacts and are not usable by the Windows policy environment.

Important Windows-side files include:

```text
windows_vlm_bridge/fr3_action_protocol.py
windows_vlm_bridge/fr3_remote_session_control.py
windows_vlm_bridge/bounded_joint_action_session.py
windows_vlm_bridge/static_vla_execution_session.py
windows_vlm_bridge/closed_loop_vla_execution_session.py
```

Do not overwrite the working `C:\vlm_bridge` automatically. Extract the
handoff into `C:\fr3_handoff`, compare it with the active files, and merge only
intentional changes.

## Prompt For Windows Codex

```text
Read docs/windows_codex_smolvla_handoff.md completely before making changes.

I am evaluating SmolVLA on this Windows GPU computer for a real Franka
Research 3 project. Ubuntu already owns all real-robot safety validation,
MoveIt planning, ros2_control execution, libfranka communication, and measured
post-execution verification.

For this task, work OFFLINE ONLY. Do not contact 10.10.10.1, ROS, MoveIt,
Isaac Sim, or the real robot. Do not run bounded sessions and do not use any
real-execution flag.

First inspect the Windows environment and the copied project files. Create a
separate SmolVLA/LeRobot environment without altering the working Qwen or GR00T
environment. Then implement the smallest raw SmolVLA inference smoke test that:

- loads the official base checkpoint;
- accepts one saved image, a dummy seven-value state, and an instruction;
- performs one inference call;
- prints and saves input feature metadata, raw output shape, action horizon,
  finite-value checks, value range, latency, and GPU memory;
- saves the untouched raw action and checkpoint/config metadata;
- clearly labels the output as NOT SAFE OR SEMANTICALLY VALID FOR FR3;
- contains no Ubuntu or robot network code.

Do not create an FR3 action adapter yet. Do not assume seven output values are
fr3_joint1 through fr3_joint7. Report the exact checkpoint input/output schema
and any camera-count requirements before proposing the next step.
```

## Transfer Instructions

On Ubuntu, serve the prepared handoff directory:

```bash
cd ~/franka_ros2_ws
python3 -m http.server 8081 --bind 10.10.10.1 --directory handoff
```

On Windows PowerShell:

```powershell
New-Item -ItemType Directory -Force C:\fr3_handoff | Out-Null

Invoke-WebRequest `
  "http://10.10.10.1:8081/fr3_windows_smolvla_handoff_2026-08-19.tar.gz" `
  -OutFile C:\fr3_handoff\fr3_windows_smolvla_handoff_2026-08-19.tar.gz

tar -xzf C:\fr3_handoff\fr3_windows_smolvla_handoff_2026-08-19.tar.gz `
  -C C:\fr3_handoff
```

Stop the temporary Ubuntu HTTP server with `Ctrl+C`. Open `C:\fr3_handoff` as
the Windows Codex workspace and paste the prompt above into a new thread.


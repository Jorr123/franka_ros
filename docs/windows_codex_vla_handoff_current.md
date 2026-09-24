# Windows Codex Handoff: FR3 VLA Deployment Project

Date: July 22, 2026

## Transfer This Handoff To Windows

On Ubuntu:

```bash
python3 -m http.server 8081 \
  --bind 10.10.10.1 \
  --directory ~/franka_ros2_ws/docs
```

On Windows PowerShell:

```powershell
Invoke-WebRequest `
  "http://10.10.10.1:8081/windows_codex_vla_handoff_current.md?v=1" `
  -OutFile C:\vlm_bridge\windows_codex_vla_handoff_current.md
```

Stop the temporary Ubuntu server with `Ctrl+C`. Attach the downloaded file to
Windows Codex and ask it to follow the `First Response Requested From Windows
Codex` section.

## Message To Windows Codex

Read this entire document before changing code. The project goal is now clear:

```text
Deploy a real VLA/policy on Windows that proposes bounded actions for a real
Franka Research 3. Ubuntu remains responsible for validation, MoveIt planning,
execution through ros2_control, and measured post-execution verification.
```

The immediate task is **policy shadow inference and the policy action adapter**.
Do not build an Isaac command-preview robot. Do not connect an Isaac camera yet.
Do not send a new policy output to Ubuntu or the real robot until its exact
action semantics and normalization are known.

## End Goal

```text
real or simulated RGB observation
+ language instruction
+ current FR3 state
        |
        v
VLA/policy on Windows
        |
        v
model-specific action adapter
        |
        v
explicit bounded FR3 action proposal
        |
        v
authenticated Windows-to-Ubuntu session protocol
        |
        v
Ubuntu safety validation
        |
        v
MoveIt plan and execution
        |
        v
ros2_control -> franka_hardware -> libfranka -> real FR3
        |
        v
fresh measured /joint_states verification
        |
        +--------------------> next policy observation
```

Windows proposes. Ubuntu decides whether the proposal is allowed and how the
real robot reaches it.

## Computers And Networks

```text
Windows PC
  IP: 10.10.10.2/24
  Runs Qwen/VLA and later Isaac Sim or Isaac Lab
          |
          | dedicated direct Ethernet
          v
Ubuntu mini PC
  Windows-facing IP: 10.10.10.1/24
  Separate robot-facing interface: 172.16.0.x/24
  Runs ROS 2 Humble, MoveIt, ros2_control, and Franka software
          |
          v
FR3 robot: 172.16.0.2
```

The two networks must remain separate.

## What Has Been Completed

### 1. Real-To-Sim Synchronization

The real FR3 publishes `/joint_states` on Ubuntu. Isaac Sim on Windows has
already subscribed to this topic and mirrored physical FR3 movement correctly:

```text
real FR3 -> Ubuntu /joint_states -> Isaac Sim FR3
```

The real robot is the authoritative joint-state source. Isaac must not publish
a second `/joint_states` into the same ROS domain.

### 2. Safe Real-Robot Motion Through MoveIt

MoveIt successfully plans and moves the real FR3 through:

```text
MoveIt -> ros2_control -> franka_hardware -> libfranka -> FR3
```

Positive and negative `fr3_joint3` deltas have both passed. Conservative tested
execution values include:

```text
per-step delta:       0.03 rad
moving joints:        1
velocity scaling:     0.03
acceleration scaling: 0.03
goal tolerance:       0.002 rad
```

For a new model or new integration path, return to `0.01 rad` first.

### 3. Post-Execution Verification

After MoveIt reports success, Ubuntu reads fresh measured joint state and checks:

```text
commanded direction
minimum real movement
target error
non-commanded joint drift
settled joint velocity
fresh joint-state receipt
```

Verified positive and negative movements both passed.

### 4. Constrained Qwen Vision-To-Robot Prototype

Qwen2.5-VL runs on Windows behind an OpenAI-compatible server. The tested
prototype does this:

```text
saved or synthetic image
-> Qwen classifies red marker as left/right/unclear
-> deterministic server code maps the observation to one joint delta
-> Ubuntu validates
-> MoveIt executes
-> measured movement is verified
```

The robot successfully moved in both positive and negative `fr3_joint3`
directions based on Qwen decisions.

This is a constrained VLM controller, not yet a general VLA. Qwen chooses the
visual observation; deterministic code currently chooses the exact joint and
delta magnitude.

### 5. Authenticated Windows-Ubuntu Protocol

Implemented and tested protections include:

```text
HMAC signatures
allowed Windows client IP
fresh timestamps
unique request IDs
replay rejection
bounded request bodies
allowed-joint restriction
per-step delta limit
cumulative delta limit
maximum step count
session deadline
one action in flight
no automatic retries
terminate on failure
append-only Ubuntu and Windows logs
```

### 6. Bounded Multi-Step Sessions

Static-image and synthetic closed-loop sessions have run successfully. Each
next action waited for `VERIFIED_SUCCESS` from the real FR3.

Completed examples include:

```text
right image -> fr3_joint3 positive -> verified
left image  -> fr3_joint3 negative -> verified
```

A ten-image session executed seven verified steps before input exhaustion. A
synthetic visual environment also updated its next image only from measured
robot feedback rather than the requested action.

### 7. Windows-Requested Session Creation

The latest Ubuntu code optionally allows Windows to request creation of a
bounded session. It is disabled by default.

Ubuntu must explicitly start the supervisor with:

```text
--enable-remote-session-arm
--remote-arm-joint fr3_joint3
```

Ubuntu still fixes all maximum limits. Windows can request smaller limits but
cannot increase them or select another joint.

New files/functions:

```text
windows_vlm_bridge/fr3_remote_session_control.py
fr3_action_protocol.create_session_arm_envelope
Ubuntu endpoint: /v1/sessions/arm
```

The new code passed the complete 106-test Ubuntu software suite. The remote-arm
endpoint has not yet been tested across the actual Windows-Ubuntu network or
with the physical robot.

## Important Active-Windows Configuration Warning

The Ubuntu repository copy currently contains these Qwen defaults:

```python
VISUAL_ACTION_JOINT = "fr3_joint7"
VISUAL_ACTION_STEP = 0.01
MAX_ABS_ACTION_DELTA = 0.05
```

However, the successful real-robot tests used a Windows copy that had been
modified to produce approximately:

```text
right -> fr3_joint3 +0.03 rad
left  -> fr3_joint3 -0.03 rad
```

Before replacing `C:\vlm_bridge\qwen_vlm_server.py`, inspect its current
constants. Do not overwrite the working Windows version with the Ubuntu copy
unless the mapping change is intentional and all limits are updated together.

Check on Windows:

```powershell
Select-String -Path C:\vlm_bridge\qwen_vlm_server.py `
  -Pattern "VISUAL_ACTION_JOINT|VISUAL_ACTION_STEP|MAX_ABS_ACTION_DELTA"
```

## Failed Approaches That Must Not Be Reintroduced

Earlier raw/generic joint streaming caused Franka reflex faults, including:

```text
joint_motion_generator_velocity_discontinuity
joint_motion_generator_acceleration_discontinuity
controller_torque_discontinuity
```

Do not:

```text
call libfranka directly from Windows
stream raw policy values to hardware command interfaces
use isaac_joint_stream_controller during MoveIt tests
run two ros2_control_node instances
assume a generic seven-value output means the seven FR3 joints
let Isaac publish a duplicate authoritative /joint_states topic
```

MoveIt is the selected real-robot execution layer for the current project.

## Current Project Decision

The end goal is VLA-to-real deployment, not manual digital-twin teleoperation.
Therefore, a separate Isaac `command_preview` robot is not required.

Isaac remains useful later for:

```text
virtual camera observations
simulation-only policy testing
dataset generation
domain randomization
Isaac Lab policy training or evaluation
collision and failure regression tests
```

It is not the immediate dependency.

## Immediate Next Checkpoint

Build a **policy shadow-inference adapter on Windows**:

```text
one saved image
+ one language instruction
+ optional recorded FR3 joint state
        |
        v
selected real VLA/policy checkpoint
        |
        v
raw model action
        |
        v
model-specific adapter
        |
        v
bounded explicit action JSON
        |
        v
log only
```

No Ubuntu request and no robot motion are allowed in this checkpoint.

### Required Information Before Coding The Adapter

Record:

```text
model family:
exact checkpoint ID/path:
training or fine-tuning dataset:
required image size and preprocessing:
language prompt format:
whether proprioceptive state is required:
raw action-vector length:
meaning and ordering of every action dimension:
normalization range:
dataset statistics or unnormalization key:
action frequency used during training:
gripper convention:
```

Do not infer these details from vector length.

For example, a seven-dimensional model output may represent:

```text
[dx, dy, dz, droll, dpitch, dyaw, gripper]
```

It must not automatically be interpreted as:

```text
[fr3_joint1, fr3_joint2, ..., fr3_joint7]
```

### Recommended Software Separation

Implement independent interfaces:

```text
FrameSource
  saved image now
  Isaac camera later
  physical camera finally

PolicyBackend
  loads the exact VLA checkpoint
  returns raw action and metadata

ActionAdapter
  applies the checkpoint's documented unnormalization
  converts to physical units
  rejects unknown/nonfinite outputs
  produces a bounded explicit proposal

Executor
  disabled during shadow mode
  later uses the existing authenticated Ubuntu protocol
```

Suggested first Windows file:

```text
C:\vlm_bridge\vla_policy_shadow_test.py
```

Suggested first output log:

```json
{
  "model": "EXACT_CHECKPOINT",
  "instruction": "EXACT_TASK",
  "image_sha256": "...",
  "raw_action": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "action_semantics": [
    "dx",
    "dy",
    "dz",
    "droll",
    "dpitch",
    "dyaw",
    "gripper"
  ],
  "denormalized_action": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "bounded_proposal": null,
  "execution_enabled": false
}
```

The semantics shown above are only an example. Use the selected checkpoint's
actual documented action space.

### Shadow Checkpoint Pass Conditions

```text
[ ] exact checkpoint is selected and recorded
[ ] model loads on Windows without changing the ROS environment
[ ] one saved image inference succeeds
[ ] raw action is logged before conversion
[ ] every action dimension is documented
[ ] correct dataset normalization is applied
[ ] all converted values are finite and in physical units
[ ] out-of-range values are rejected or clamped by explicit policy
[ ] repeated identical inputs are reasonably consistent
[ ] execution_enabled is false
[ ] Ubuntu is not contacted
[ ] real robot does not move
```

## Order After The Shadow Adapter Passes

```text
Checkpoint 1: actual VLA + saved image + adapter, log only
Checkpoint 2: adapter output -> Ubuntu MoveIt PLAN_ONLY
Checkpoint 3: Isaac virtual camera replaces saved image, infer-only
Checkpoint 4: simulation/shadow closed loop with no real motion
Checkpoint 5: one remotely or locally armed 0.01 rad real action
Checkpoint 6: measured verification and fresh observation
Checkpoint 7: two-step bounded closed loop
Checkpoint 8: physical camera shadow mode
Checkpoint 9: physical-camera bounded closed loop
```

The Isaac camera comes after the action adapter because image acquisition and
action interpretation are independent. A camera provides pixels; it does not
define what a model's output means.

## Remote Session Arm Files To Transfer

The updated Windows files are available on Ubuntu under:

```text
/home/itri/franka_ros2_ws/windows_vlm_bridge/fr3_action_protocol.py
/home/itri/franka_ros2_ws/windows_vlm_bridge/fr3_remote_session_control.py
```

Temporary transfer from Ubuntu:

```bash
python3 -m http.server 8081 \
  --bind 10.10.10.1 \
  --directory ~/franka_ros2_ws/windows_vlm_bridge
```

Windows PowerShell:

```powershell
Invoke-WebRequest `
  "http://10.10.10.1:8081/fr3_action_protocol.py?v=remote-arm-1" `
  -OutFile C:\vlm_bridge\fr3_action_protocol.py

Invoke-WebRequest `
  "http://10.10.10.1:8081/fr3_remote_session_control.py?v=remote-arm-1" `
  -OutFile C:\vlm_bridge\fr3_remote_session_control.py
```

Verify:

```powershell
cd C:\vlm_bridge
python -c "import fr3_action_protocol as p; print(p.__file__); print(hasattr(p, 'create_session_arm_envelope'))"
```

Expected final output:

```text
True
```

Remote arm/status/stop alone never moves the robot. A session step submitted by
an execution client is required for motion.

## Existing Windows Files

The Windows working directory is normally:

```text
C:\vlm_bridge
```

Expected existing files include:

```text
qwen_vlm_server.py
fr3_action_protocol.py
fr3_remote_session_control.py
isaac_qwen_coordinator.py
static_vla_shadow_session.py
static_vla_execution_session.py
closed_loop_vla_shadow_session.py
closed_loop_vla_execution_session.py
```

Inspect existing files before replacing them because the active Windows copies
may contain intentional configuration changes not present in Ubuntu.

## Existing Ubuntu Files

Workspace:

```text
/home/itri/franka_ros2_ws
```

Important scripts:

```text
scripts/fr3_ai_supervisor.py
scripts/fr3_bounded_session_supervisor.py
scripts/fr3_session_control.py
scripts/joint_motion_verification.py
scripts/qwen_vlm_joint_action_test.py
scripts/vla_joint_command_test.py
```

Important documentation:

```text
docs/windows_remote_session_arm.md
docs/bounded_static_execution_session.md
docs/synthetic_hardware_loop.md
docs/isaac_virtual_wrist_camera_hil.md
docs/july_21_progress.md
```

## Ubuntu Runtime Commands For Later

These are not needed for the immediate policy shadow checkpoint.

MoveIt:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2
```

The bounded supervisor can run local-only or with explicitly enabled remote
session arm. Do not start it during pure Windows policy shadow inference.

## Current Unresolved Items

```text
No true VLA checkpoint has been selected yet.
No VLA action semantics or normalization have been documented yet.
No VLA action adapter exists yet.
No physical camera is installed yet.
Isaac virtual-camera frames are not connected to the policy pipeline yet.
Remote Windows arm is software-tested but not network/hardware-tested yet.
The active Windows Qwen constants may differ from the Ubuntu source copy.
```

## Safety Boundary For Windows Codex

During the immediate checkpoint, do not:

```text
start MoveIt execution
arm a real session
send HTTP requests to Ubuntu session endpoints
modify libfranka or ros2_control
reinterpret unknown VLA outputs as joint commands
install model dependencies into the working Qwen environment without first
checking for version conflicts
```

Prefer a separate Windows environment for the selected VLA if its Torch,
Transformers, CUDA, or model dependencies differ from the working Qwen setup.

## First Response Requested From Windows Codex

After reading this handoff, Windows Codex should:

1. Inspect `C:\vlm_bridge` without overwriting files.
2. Record the active Qwen constants and installed environment versions.
3. Ask for or confirm the exact VLA model/checkpoint.
4. Inspect that checkpoint's official action-space and normalization contract.
5. Propose the minimal `vla_policy_shadow_test.py` design.
6. Keep execution disabled and Ubuntu disconnected.

The first deliverable is one reproducible infer-only log containing the raw
policy action and a correctly documented, bounded conversion.

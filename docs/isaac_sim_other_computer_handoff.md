# FR3, Isaac Sim, Qwen, and MoveIt Project Handoff

## Transfer This Handoff To Windows

Temporarily serve the Ubuntu documentation directory:

```bash
python3 -m http.server 8081 \
  --bind 10.10.10.1 \
  --directory ~/franka_ros2_ws/docs
```

Download this file in Windows PowerShell:

```powershell
Invoke-WebRequest `
  http://10.10.10.1:8081/isaac_sim_other_computer_handoff.md `
  -OutFile C:\vlm_bridge\isaac_sim_other_computer_handoff.md
```

Stop the temporary Ubuntu file server with `Ctrl+C`. Attach the downloaded
Markdown file to ChatGPT on Windows, or paste its contents into the first
message.

## Instructions For The Assistant On The Windows Computer

I am continuing this project on the Windows PC where Isaac Sim is installed.
Please read this document before suggesting code or configuration changes.

The immediate task is **only to add and verify an Isaac Sim virtual wrist
camera**. Do not redesign the Ubuntu robot-control path, do not command the
real FR3 directly from Isaac, and do not bypass MoveIt or `ros2_control`.

Before giving version-specific Isaac Python code, first ask me for:

```text
Isaac Sim version:
FR3 articulation prim path:
FR3 hand/tool prim path:
existing ROS Action Graph path or screenshot:
```

The camera API and extension names can differ between Isaac Sim versions. Use
the APIs installed with my exact Isaac Sim version.

## Project Goal

Build a safe vision-to-robot system:

```text
Isaac Sim or a future camera/policy on Windows
  -> Qwen/VLA proposes a small robot action
  -> Windows validates and signs the proposal
  -> Ubuntu validates the bounded session and action
  -> MoveIt plans and executes
  -> ros2_control -> franka_hardware -> libfranka
  -> real FR3 moves
  -> measured /joint_states verifies the result
  -> Isaac Sim follows the new measured robot state
```

Windows proposes actions. Ubuntu owns robot safety, planning, execution, and
post-execution verification.

## Computers And Networks

```text
Windows PC
  Windows-Ubuntu Ethernet: 10.10.10.2/24
  Runs Qwen2.5-VL and Isaac Sim
          |
          | direct Ethernet, HTTP and ROS 2 traffic
          v
Ubuntu mini PC
  Windows-Ubuntu Ethernet: 10.10.10.1/24
  Robot Ethernet: a separate 172.16.0.x/24 interface
  Runs ROS 2 Humble, MoveIt, ros2_control, and Franka software
          |
          v
FR3 robot: 172.16.0.2
```

The `10.10.10.x` and `172.16.0.x` networks must remain separate.

## What Already Works

### Real To Sim

The real FR3 publishes measured joint state on Ubuntu. Isaac Sim subscribes to
`/joint_states`, and its FR3 articulation follows the physical robot.

```text
real FR3 -> Ubuntu /joint_states -> Isaac Sim FR3
```

Physical robot movement has already been mirrored correctly in Isaac Sim.

### MoveIt Robot Control

MoveIt successfully plans and moves the real FR3 through the official stack:

```text
MoveIt -> ros2_control -> franka_hardware -> libfranka -> FR3
```

Small positive and negative `fr3_joint3` movements have passed. Conservative
tested values are:

```text
per-action delta:       0.03 rad
moving joints:          1
velocity scaling:       0.03
acceleration scaling:   0.03
goal tolerance:         0.002 rad
```

For the first Isaac-camera hardware test, reduce the per-action delta to
`0.01 rad`.

### Qwen Vision And Windows-Ubuntu Bridge

Qwen2.5-VL runs on Windows behind an OpenAI-compatible HTTP server. It has
correctly classified test images containing a red marker as left, right, or no
valid marker.

The current experiment is deliberately constrained:

```text
Qwen observes left/right/unclear
  -> deterministic code maps the observation to one joint delta
  -> safety validation
  -> MoveIt
```

This is a VLM-driven controller test, not yet a general VLA. A future OpenVLA,
GR00T, or Isaac Lab policy will require an adapter to the same bounded action
contract.

### Safety And Verification

The following mechanisms have been implemented and tested:

- HMAC-authenticated Windows-to-Ubuntu action envelopes;
- unique request IDs, replay rejection, and request expiry;
- locally bounded execution sessions;
- allowed-joint, maximum-step, deadline, and cumulative-delta limits;
- one action in flight and no automatic retries;
- MoveIt planning and collision checking;
- low velocity and acceleration scaling;
- measured post-execution `/joint_states` verification;
- direction, target error, other-joint drift, and settled-velocity checks; and
- terminate-on-failure behavior.

Static image sessions and synthetic closed-loop sessions have already moved the
real robot successfully in both positive and negative `fr3_joint3` directions.

## Earlier Approaches That Should Not Be Repeated

Direct or generic joint streaming produced Franka discontinuity faults,
including motion-generator velocity/acceleration discontinuities and torque
discontinuities. A custom streaming controller was explored, but MoveIt became
the selected execution layer because it plans trajectories and has already
worked reliably with the real robot.

Do not:

- call `libfranka` directly from Windows or from a new ROS node;
- stream raw Isaac joint targets directly to Franka command interfaces;
- run the old `isaac_joint_stream_controller` during MoveIt tests;
- start two `ros2_control_node` instances; or
- let Isaac publish a second authoritative `/joint_states` topic.

## ROS Topic Ownership

The real robot is the only authoritative joint-state source:

```text
joint_state_broadcaster on Ubuntu
  -> /joint_states
  -> Isaac Sim subscriber
```

On Ubuntu, verify ownership with:

```bash
ros2 topic info /joint_states --verbose
```

Isaac must be a subscriber only. If simulated joint states are needed for
debugging, use a separate topic such as:

```text
/isaac_sim/debug/joint_states
```

Always map joints by name, never only by array index.

## Immediate Isaac Sim Camera Task

The first camera checkpoint is:

```text
real measured /joint_states
  -> existing Isaac FR3 follower
  -> virtual camera parented to the wrist/tool link
  -> camera view changes when the real robot moves
```

Qwen and real robot action generation must remain disconnected while this
checkpoint is being built.

### Step 1: Save And Inspect The Existing Stage

1. Open the Isaac stage in which the FR3 already follows `/joint_states`.
2. Save a backup copy before changing the stage.
3. Record the Isaac Sim version.
4. In the Stage tree, record the FR3 articulation root prim path.
5. Find and record the moving hand or tool-link prim path.
6. Open **Window > Graph Editors > Action Graph** and identify the graph used
   for real-to-sim following.

The existing graph should conceptually contain:

```text
On Playback Tick
  -> ROS2 Subscribe Joint State (/joint_states)
  -> Articulation Controller (existing FR3 articulation)
```

The subscriber's joint names and position command should feed the articulation
controller. Do not connect effort commands.

### Step 2: Audit `/joint_states`

With Isaac playing, run on Ubuntu:

```bash
ros2 topic info /joint_states --verbose
```

Pass conditions:

```text
exactly one real publisher, normally joint_state_broadcaster
Isaac appears as a subscriber
no Isaac publisher exists on /joint_states
```

Do not continue until this is true.

### Step 3: Create The Virtual Wrist Camera

Pause the Isaac timeline first.

1. In the Stage tree, select the FR3 hand or tool-link prim.
2. Create an Xform child named `wrist_camera_mount` if a mount Xform does not
   already exist.
3. Create a Camera prim named `wrist_camera` under that mount.
4. Confirm this hierarchy in the Stage tree:

```text
FR3 hand/tool link
  -> wrist_camera_mount (Xform)
       -> wrist_camera (Camera)
```

5. Reset or inspect the child's local transform so an old world-space
   transform is not accidentally retained.
6. Give the mount a small local translation that places the camera outside the
   gripper geometry.
7. Rotate it toward the work area. USD/Omniverse cameras look along local
   negative Z, with local positive Y as image-up.
8. Set a moderate field of view and clipping range.
9. Start with `640 x 480` output.
10. Select `wrist_camera` from the viewport camera menu and inspect the image.

Do not guess a final calibration transform. First find a collision-free virtual
mount that gives a clear view, then record:

```text
camera prim path:
parent/mount prim path:
local translation:
local orientation:
focal length or field of view:
clipping range:
resolution:
```

### Step 4: Verify Camera Parenting

Press Play. Use the already working real-to-sim follower and make only one tiny
MoveIt-planned real motion with the physical workspace clear.

Pass conditions:

```text
real FR3 moves
Isaac FR3 follows the measured joint state
wrist camera moves with the hand/tool link
camera viewpoint changes consistently
stopping Isaac playback does not command the real robot
```

If the robot follows but the camera does not, the camera is probably under a
static prim or the wrong link. Inspect the Stage hierarchy and local transform.

### Step 5: Add A Simple Virtual Target

1. Add a small sphere, disk, or cube in front of the camera.
2. Give it a bright red material.
3. Keep it virtual and non-colliding during the first perception test.
4. Keep the corresponding real workspace empty.
5. If a center guide is required, add it as a 2D image overlay during frame
   export, not as a 3D object assumed to stay at the image center.

Manually capture these three camera frames:

```text
target_left.png
target_right.png
target_center.png
```

The images must be sharp, correctly oriented, and clearly show the marker.

### Step 6: Run Qwen Inference Only

Keep the image and Qwen request on Windows. The existing Qwen server is:

```powershell
cd C:\vlm_bridge
conda activate vlm
python -m uvicorn qwen_vlm_server:app `
  --host 0.0.0.0 `
  --port 8000 `
  --workers 1
```

Expected repeated observations:

```text
target_right.png  -> right
target_left.png   -> left
target_center.png -> NO_ACTION or centered
```

No Ubuntu request and no robot execution are allowed in this step.

### Step 7: Measure The Camera-To-Joint Mapping In Simulation

Do not assume that right means positive `fr3_joint3`. Camera mounting and robot
pose can reverse the image response.

In a simulation-only copy/mode, disconnected from real execution:

1. Record marker center pixel `x_before`.
2. Apply `+0.01 rad` to simulated `fr3_joint3` only.
3. Render a fresh frame and record `x_after`.
4. If `x_after < x_before`, positive joint 3 moved the marker left in the
   image. If `x_after > x_before`, it moved the marker right.
5. Restore the pose and repeat with `-0.01 rad`.
6. Confirm the response is opposite and repeatable.

Do this in a duplicate stage or temporarily disable the real `/joint_states`
follower. Never command the simulated articulation and the measured-state
follower at the same time. Restore the follower before hardware-in-the-loop
testing.

The measured mapping is valid only near the tested robot pose and camera
transform.

## Camera Adapter To Build After The GUI Checkpoint

Only after the camera follows correctly and saved images pass Qwen should a
Windows camera adapter be written.

The adapter should provide a camera-independent interface similar to:

```python
frame = frame_source.get_latest_rgb()

# Required metadata:
# frame.rgb: H x W x 3 uint8 RGB image
# frame.index: strictly increasing frame number
# frame.timestamp: capture time
# frame.sha256: hash of the exact image used for inference
# frame.camera_prim: Isaac camera prim path
```

Required behavior:

- acquire RGB from the exact `wrist_camera` render product;
- reject empty, stale, or duplicate frames;
- preserve frame index, timestamp, and SHA-256 in the action/session log;
- call Qwen locally at `http://127.0.0.1:8000`;
- send only the small validated action envelope to Ubuntu;
- never publish robot commands directly from the camera adapter; and
- wait for the Ubuntu result before requesting another frame/action.

The exact Isaac import path and frame-capture API must be selected after
checking the installed Isaac Sim version. Prefer an in-process Isaac extension
or Script Editor prototype first. Keep the frame-source interface independent
so it can later be replaced by a physical camera.

## Safe Integration Order After Camera Capture Works

### Checkpoint A: Camera Only

```text
real FR3 -> Isaac follower -> wrist camera image
```

No Qwen action and no robot command.

### Checkpoint B: Infer Only

```text
fresh Isaac frame -> local Qwen -> logged observation/action
```

No Ubuntu request and no robot command.

### Checkpoint C: Ubuntu Plan Only

```text
fresh Isaac frame -> Qwen -> signed action -> Ubuntu PLAN_ONLY -> MoveIt plan
```

Ubuntu health must report:

```text
mode: PLAN_ONLY
execution_available: false
```

The result must be `PLAN_READY`, and neither robot may move.

### Checkpoint D: One Bounded Real Step

Locally arm one action for `fr3_joint3`, limited to `0.01 rad`. Require:

```text
planning success
execution success
measured direction pass
target error pass
other-joint drift pass
settled velocity pass
Isaac receives the new measured state
a fresh camera image is rendered
visual error decreases
```

End the session after this single step.

### Checkpoint E: Two-Step Closed Loop

Allow two actions at `0.01 rad` each. A new action is permitted only after:

```text
Ubuntu returns VERIFIED_SUCCESS
Isaac receives a fresh real joint state
the wrist camera produces a new frame index and hash
the previous image error decreased
```

Stop on low confidence, NO_ACTION, wrong visual direction, stale frame, plan or
execution failure, verification failure, timeout, or exhausted limits.

Only after repeated success should the tested `0.03 rad` per-step limit be
considered again.

## Ubuntu Commands For Later Checkpoints

Start the one real MoveIt stack:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2
```

The bounded supervisor and session-control scripts already exist:

```text
scripts/fr3_bounded_session_supervisor.py
scripts/fr3_session_control.py
```

Do not start execution while setting up the camera. The initial Windows-side
work should stop at camera-only and infer-only checkpoints.

## Existing Project Files

Ubuntu workspace root:

```text
/home/itri/franka_ros2_ws
```

Important Ubuntu scripts:

```text
scripts/fr3_ai_supervisor.py
scripts/fr3_bounded_session_supervisor.py
scripts/fr3_session_control.py
scripts/joint_motion_verification.py
scripts/qwen_vlm_joint_action_test.py
scripts/vla_joint_command_test.py
```

Windows bridge source copies in the Ubuntu workspace:

```text
windows_vlm_bridge/qwen_vlm_server.py
windows_vlm_bridge/fr3_action_protocol.py
windows_vlm_bridge/isaac_qwen_coordinator.py
windows_vlm_bridge/static_vla_execution_session.py
windows_vlm_bridge/closed_loop_vla_shadow_session.py
windows_vlm_bridge/closed_loop_vla_execution_session.py
```

The active Windows copies are normally under:

```text
C:\vlm_bridge
```

Detailed Ubuntu documentation:

```text
docs/july_21_progress.md
docs/isaac_virtual_wrist_camera_hil.md
docs/windows_isaac_plan_bridge.md
docs/bounded_static_execution_session.md
docs/mock_visual_closed_loop.md
docs/synthetic_hardware_loop.md
```

## Current Limitations

- There is no physical camera yet.
- The Isaac camera sees only the simulated scene.
- A virtual target does not prove alignment with a real target.
- The current red-marker experiment is a constrained VLM controller, not a
  general VLA.
- A future VLA action vector cannot automatically be treated as FR3 joint
  deltas; it needs an embodiment-specific adapter and validation.
- The one-joint visual mapping may change with robot pose or camera transform.

## Definition Of Success For The Current Windows Session

Stop after these boxes are complete:

```text
[ ] Isaac Sim version and relevant prim paths are recorded
[ ] Existing real-to-sim graph is unchanged and still works
[ ] Ubuntu reports one authoritative /joint_states publisher
[ ] wrist_camera is parented to the moving hand/tool link
[ ] camera view follows one tiny real MoveIt movement
[ ] red virtual target is visible
[ ] left, right, and centered PNG frames are saved
[ ] Qwen classifies all three correctly in infer-only mode
[ ] no action from Isaac can move the real FR3 during camera setup
```

After these pass, the next deliverable is the version-specific Isaac RGB frame
adapter, first in infer-only mode.

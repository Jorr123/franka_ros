# Isaac Virtual Wrist Camera Hardware-In-The-Loop Runbook

## Objective

Build this pipeline in small, independently testable stages:

```text
real FR3 measured joint states
  -> Isaac Sim digital twin
  -> virtual wrist-camera image
  -> Qwen action proposal on Windows
  -> Ubuntu safety validation and MoveIt
  -> real FR3 motion
  -> new measured joint states
  -> new Isaac camera image
```

The virtual camera sees only the virtual Isaac scene. It does not observe the
real workspace. Until a physical camera is installed and calibrated, this is a
hardware-in-the-loop integration test rather than real visual manipulation.

## Safety Rules

1. Keep the real workspace clear and keep the Franka user stop within reach.
2. Do not run `isaac_joint_stream_controller` during these tests.
3. Run only one `ros2_control_node`; use the one started by the MoveIt launch.
4. The real robot is the only authoritative joint-state source.
5. Isaac may subscribe to `/joint_states`, but it must not publish that topic.
6. Never use the virtual camera to claim that an unmodeled real obstacle is safe.
7. Complete each checkpoint before proceeding to the next one.

## Stage 0: Record The Isaac Version And Existing Prim Paths

On Windows, open Isaac Sim and record:

```text
Isaac Sim version: __________________
FR3 articulation prim: __________________
hand/tool prim: __________________
```

The exact camera Python API differs between Isaac Sim versions. The GUI camera
and ROS Action Graph steps below are intentionally version-independent.

Use the existing FR3 stage that already mirrors the real robot. Do not add a
second robot.

## Stage 1: Audit The Real-To-Sim Action Graph

Open **Window > Graph Editors > Action Graph** and locate the graph that follows
the real FR3.

The required data flow is:

```text
On Playback Tick
  -> ROS2 Subscribe Joint State
  -> Articulation Controller
```

Configure it as follows:

```text
ROS2 Subscribe Joint State topic: /joint_states
Articulation Controller target:   the existing FR3 articulation prim
```

Connect these subscriber outputs to the Articulation Controller:

```text
jointNames       -> jointNames
positionCommand  -> positionCommand
```

Do not connect velocity or effort commands for this measured-position follower.

Disable or delete these nodes if they target the real ROS domain:

```text
ROS2 Publish Joint State publishing /joint_states
any Isaac command publisher targeting the real robot
```

If simulated joint states are needed for debugging later, publish them under a
different name such as `/isaac_sim/debug/joint_states`.

### Checkpoint 1A: ROS Publisher Audit

On Ubuntu, with Isaac playing:

```bash
ros2 topic info /joint_states --verbose
```

There should be one real joint-state publisher, normally
`joint_state_broadcaster`. Isaac should appear only as a subscriber.

### Checkpoint 1B: State Following

Start the normal real MoveIt stack on Ubuntu:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2
```

Use RViz or the already-tested joint-goal script to perform one tiny, planned
movement. Confirm that:

1. The real FR3 moves.
2. The Isaac FR3 follows it.
3. Stopping the Isaac timeline cannot command the real FR3.

Do not continue if Isaac publishes `/joint_states`, the joint names do not map
correctly, or the simulated pose differs visibly from the measured real pose.

## Stage 2: Attach A Virtual Wrist Camera

Pause the Isaac timeline.

1. In the Stage tree, locate the FR3 hand or tool link.
2. Select **Create > Camera**.
3. Name it `wrist_camera`.
4. Reparent it under the hand/tool prim by dragging it in the Stage tree.
5. Set a small local translation that places it just outside the gripper body.
6. Orient it toward the workspace. Omniverse cameras look along their local
   negative Z axis, with local positive Y as image-up.
7. Set a moderate field of view and clipping range.
8. In the viewport camera menu, select `wrist_camera`.

Do not guess a permanent camera transform yet. Adjust it visually so the tool
and workspace are visible, then record the local camera transform:

```text
parent prim: __________________
local translation: __________________
local orientation: __________________
resolution: 640 x 480
```

### Checkpoint 2: Camera Attachment

Press Play and execute one tiny real MoveIt movement. The simulated FR3 and
camera must move together. The view must change because the camera inherited
the hand/tool transform.

Pause Isaac and move the FR3 prim manually only for this diagnostic: if the
camera does not follow, it was not parented to the moving link correctly. Undo
the manual edit afterward and restore measured-state following.

## Stage 3: Add A Virtual Visual Target

Keep this first scene deliberately simple:

1. Add a small sphere or disk in front of the camera.
2. Give it a bright red material.
3. If Qwen needs a visible center reference, draw a thin vertical overlay on
   the exported image. Do not place a 3D line in the scene and assume it stays
   at the image center.
4. Put the target outside the center deadband but clearly inside the image.
5. Keep the target non-colliding for this perception-only checkpoint.

The target exists only in simulation. Keep the corresponding real workspace
empty.

### Checkpoint 3

Capture three images manually from the wrist-camera viewport:

```text
target_left.png
target_right.png
target_center.png
```

Inspect the files before using Qwen. The marker must be sharp, correctly colored,
and clearly separated from the center line.

## Stage 4: Qwen Infer-Only Test

Keep MoveIt execution disconnected. Feed each captured image to Qwen on Windows
using the existing local Qwen endpoint and validator.

Expected decisions:

```text
target_right.png  -> observation: right
target_left.png   -> observation: left
target_center.png -> NO_ACTION
```

Do not assume which joint-delta sign corresponds to left or right. That mapping
depends on the camera extrinsics and current robot configuration. Determine it
in the simulation-only response test below and then let deterministic code map
the Qwen observation to the bounded delta.

Use no more than `0.01 rad` for the first virtual-camera test. The `0.03 rad`
limit can be restored only after the sign and camera response have been checked.

### Checkpoint 4

Qwen must classify all three saved frames correctly in repeated infer-only runs.
No Ubuntu request and no robot motion are allowed in this stage.

## Stage 5: Move The Simulated Robot Only

Before using the real FR3, close the loop entirely inside Isaac:

```text
Isaac wrist frame
  -> Qwen
  -> validated tiny delta
  -> simulated FR3 only
  -> fresh wrist frame
```

Use a dedicated simulated action topic or direct Isaac articulation command.
Never reuse the real `/joint_states` follower graph for simulated command input.
The real ROS/MoveIt execution endpoint must remain disabled.

Before closing the loop, measure the visual response:

1. Record the target center pixel `x_before`.
2. Apply `+0.01 rad` to `fr3_joint3` in simulation only.
3. Render a fresh frame and record `x_after`.
4. If `x_after < x_before`, positive joint 3 moves the marker left in the
   current setup. If `x_after > x_before`, it moves the marker right.
5. Repeat with `-0.01 rad` and verify the opposite response.
6. Restore the original simulated pose.

Keep this mapping valid only for the tested pose region. Stop if the measured
visual response changes sign or becomes too small.

The loop terminates on any of:

```text
marker centered
NO_ACTION or low confidence
wrong-direction visual response
stale or duplicate frame
two actions reached
0.02 rad cumulative motion reached
timeout
```

### Checkpoint 5

The image error must decrease after each simulated action. Save every frame,
action, frame hash, and observed error to a session log.

## Stage 6: Ubuntu Plan-Only Test

Restore real-to-sim measured-state following. Do not command the simulated robot
independently in this mode.

Run the existing Ubuntu plan-only supervisor and send one action produced from a
fresh Isaac camera frame:

```text
Isaac frame -> Qwen -> Windows validation -> Ubuntu PLAN_ONLY -> MoveIt plan
```

Confirm the Ubuntu health endpoint reports:

```text
mode: PLAN_ONLY
execution_available: false
```

### Checkpoint 6

Ubuntu returns `PLAN_READY`, the planned joint is the expected joint, and neither
the real nor simulated robot moves.

## Stage 7: One Bounded Real Step

Only after all previous checkpoints pass:

1. Start MoveIt and the bounded Ubuntu execution supervisor.
2. Locally arm exactly one action for the expected joint.
3. Limit the first action to `0.01 rad`.
4. Capture one fresh Isaac wrist frame.
5. Ask Qwen for one action.
6. Send the signed action envelope to Ubuntu.
7. Let MoveIt plan and execute on the real FR3.
8. Require Ubuntu `VERIFIED_SUCCESS` from fresh measured `/joint_states`.
9. Confirm Isaac receives the new measured position and updates the wrist view.
10. End the session. Do not automatically request a second action yet.

### Checkpoint 7

All of these must pass:

```text
MoveIt planning:             SUCCESS
MoveIt execution:            SUCCESS
measured direction:          PASS
measured target error:       PASS
non-commanded joint drift:   PASS
settled velocity:            PASS
new Isaac frame received:    PASS
visual error decreased:      PASS
```

## Stage 8: Two-Step Hardware-In-The-Loop Test

Increase the locally armed budget to two actions, still at `0.01 rad` per action.
Every next inference must wait for:

1. `VERIFIED_SUCCESS` from Ubuntu;
2. a new real joint-state sample in Isaac;
3. a newly rendered camera frame with a different timestamp and hash.

Stop immediately on the first rejected plan, execution failure, verification
failure, stale frame, low confidence, no action, or increasing visual error.

Only after repeated two-step runs pass should the per-step limit return to
`0.03 rad`.

## Final Architecture Before A Physical Camera

```text
Windows
  Isaac Sim virtual scene and wrist camera
  -> local Qwen inference
  -> local action validation
  -> signed bounded action request

Ubuntu
  bounded session supervisor
  -> current measured joint state + requested delta
  -> absolute MoveIt joint goal
  -> ros2_control
  -> libfranka
  -> real FR3
  -> measured post-execution verification

Ubuntu -> Windows
  real /joint_states
  -> Isaac digital-twin follower
  -> fresh virtual wrist image
```

## Important Limitation

The virtual target is not registered to a real target unless the digital twin is
calibrated against the real workspace. A virtual-camera success therefore means
that the software loop and kinematic synchronization worked. It does not prove
that the real robot reached or observed a real object.

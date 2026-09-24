# Synthetic Hardware-In-The-Loop Session

This checkpoint connects the synthetic visual loop to a real, locally armed
FR3 execution session:

```text
synthetic frame on Windows
  -> Qwen observes left/right
  -> signed bounded-session step
  -> Ubuntu validates and asks MoveIt to plan
  -> MoveIt executes on the FR3
  -> Ubuntu verifies fresh measured joint state
  -> Windows validates the measured receipt
  -> measured delta updates the synthetic marker
  -> next frame
```

The synthetic marker never updates from the requested action. It updates only
after a matching `VERIFIED_SUCCESS` receipt containing internally consistent
measured positions, direction, target error, non-commanded drift, and settled
velocity.

This is a hardware integration test, not real visual servoing. The mock mapping
between joint motion and marker pixels is artificial.

## Safety Boundary

The first test is limited to:

```text
joint:                  fr3_joint3
maximum steps:          2
maximum delta per step: 0.03 rad
maximum cumulative:     0.06 rad
automatic retries:      0
local arming:            required once per run
```

Keep the workspace clear, confirm the current posture has room for the proposed
joint-3 direction, and keep the Franka user stop within reach. Do not raise the
limits to make a failed test pass.

## 1. Run Software Tests On Ubuntu

```bash
cd ~/franka_ros2_ws
python3 -m unittest -v tests/test_closed_loop_vla_execution_session.py
```

Expected:

```text
Ran 10 tests
OK
```

These tests do not use ROS, MoveIt, or the robot.

## 2. Transfer Updated Windows Files

Ubuntu:

```bash
python3 -m http.server 8081 \
  --bind 10.10.10.1 \
  --directory ~/franka_ros2_ws/windows_vlm_bridge
```

Windows PowerShell:

```powershell
Invoke-WebRequest `
  "http://10.10.10.1:8081/closed_loop_vla_execution_session.py?v=1" `
  -OutFile C:\vlm_bridge\closed_loop_vla_execution_session.py

Invoke-WebRequest `
  "http://10.10.10.1:8081/static_vla_execution_session.py?v=2" `
  -OutFile C:\vlm_bridge\static_vla_execution_session.py
```

Keep the current copies of these dependencies in `C:\vlm_bridge`:

```text
closed_loop_vla_shadow_session.py
fr3_action_protocol.py
isaac_qwen_coordinator.py
static_vla_shadow_session.py
```

Stop the temporary Ubuntu file server with `Ctrl+C`.

## 3. Start Qwen On Windows

Windows terminal 1:

```powershell
cd C:\vlm_bridge
conda activate vlm
python -m uvicorn qwen_vlm_server:app `
  --host 0.0.0.0 `
  --port 8000 `
  --workers 1
```

Use the already tested mapping:

```text
right -> fr3_joint3 +0.03
left  -> fr3_joint3 -0.03
centered/unclear -> zero motion
```

## 4. Start MoveIt On Ubuntu

Ubuntu terminal 1:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2
```

Use only one MoveIt/`ros2_control` launch.

## 5. Restart The Bounded Supervisor

Stop the old supervisor before starting this updated copy. Ubuntu terminal 2:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export FR3_BRIDGE_SECRET='YOUR_EXISTING_SHARED_SECRET'

python3 scripts/fr3_bounded_session_supervisor.py \
  --enable-bounded-session-execution \
  --bind-host 10.10.10.1 \
  --allowed-client 10.10.10.2 \
  --session-max-steps 2 \
  --session-max-duration 60 \
  --session-max-cumulative-delta 0.06 \
  --max-delta 0.03 \
  --max-total-delta 0.03 \
  --max-moving-joints 1 \
  --velocity-scaling 0.03 \
  --acceleration-scaling 0.03
```

Required startup state:

```text
NO SESSION IS ARMED. THE ROBOT CANNOT MOVE FROM REMOTE REQUESTS YET.
Direct remote execution endpoint: NONE
```

## 6. Arm One Two-Step Session

After checking the robot and workspace, use Ubuntu terminal 3:

```bash
cd ~/franka_ros2_ws
python3 scripts/fr3_session_control.py arm \
  --joint fr3_joint3 \
  --max-steps 2 \
  --max-duration 60 \
  --max-delta 0.03 \
  --max-cumulative-delta 0.06
```

Review the displayed limits and type `ARM`. Windows discovers the fresh session
automatically; do not copy the session UUID.

## 7. Execute The Right-Start Test

Windows terminal 2:

```powershell
cd C:\vlm_bridge
conda activate vlm
$env:FR3_BRIDGE_SECRET = "YOUR_EXISTING_SHARED_SECRET"

python closed_loop_vla_execution_session.py `
  --start-side right `
  --start-offset-px 60 `
  --marker-radius 12 `
  --deadband-px 15 `
  --joint fr3_joint3 `
  --max-actions 2 `
  --max-delta 0.03 `
  --max-total-delta 0.03 `
  --max-cumulative-delta 0.06 `
  --max-measured-delta-error 0.005 `
  --max-session-seconds 60 `
  --interval 0.5 `
  --output-dir C:\vlm_bridge\mock_runs\hardware_right_01 `
  --run-armed-session
```

Expected progression:

```text
Observation 1: right near +60 px
  VERIFIED step 1: requested +0.030, measured approximately +0.030 rad
  marker updates from measured feedback
Observation 2: right near +30 px
  VERIFIED step 2: requested +0.030, measured approximately +0.030 rad
  marker reaches the deadband
Observation 3: centered
  Qwen returns NO_ACTION
```

Expected ending:

```text
Status:              GOAL_REACHED
Verified steps:      2
Ubuntu status:       COMPLETED
Stop confirmed:      True
```

The final marker error can be a few pixels rather than exactly zero because it
uses measured motion.

## 8. Inspect Before Testing Left

Confirm all of the following:

```text
two and only two robot movements occurred
both steps printed VERIFIED
measured and requested deltas have the same sign
the synthetic error decreased after each measured step
events.jsonl orders STEP_VERIFIED before MEASURED_MOCK_UPDATE
the final status is GOAL_REACHED
```

Any other status is a failed test. Do not arm another session until the cause is
understood.

## 9. Execute The Left-Start Test

Arm a new session with the same Ubuntu command. Then rerun the Windows command
with:

```powershell
--start-side left
--output-dir C:\vlm_bridge\mock_runs\hardware_left_01
```

The two requested and measured deltas must be negative. This should approximately
return joint 3 toward its position before the right-start test.

## Failure Behavior

`STEP_FAILED` means planning, execution, HTTP receipt validation, or measured
verification failed. There is no retry and the marker does not update.

`MEASURED_MOTION_DID_NOT_IMPROVE` means the robot moved and was verified, but
the measured direction did not reduce the mock visual error. The session stops.

`PREMATURE_NO_ACTION` or `PERCEPTION_MISMATCH` means Qwen did not satisfy the
synthetic visual contract. No additional step is sent.

`FAILED_STOP_UNCONFIRMED` means Windows could not confirm removal of remaining
session authority. Stop the Ubuntu supervisor and inspect its terminal before
continuing.

## Checkpoint Result

Pass when right and left each reach `GOAL_REACHED`, every state update follows a
measured `VERIFIED_SUCCESS`, and no retry or extra robot movement occurs.

The next stage is a replaceable frame-source interface tested with prerecorded
frames. A webcam or Isaac camera comes later; the synthetic mapping must never
be treated as camera calibration.

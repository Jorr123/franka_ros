# Windows-Requested Bounded FR3 Sessions

> This document describes the earlier one-joint session interface. The current
> schema-v2 multi-joint commands are in `docs/bounded_multi_joint_session.md`.
> Do not use the old `allowed_joint` examples with the updated supervisor.

## Purpose

This optional mode allows Windows to request the start of a bounded execution
session without typing `ARM` on Ubuntu for every run.

Ubuntu remains authoritative:

```text
Ubuntu operator starts supervisor with remote arm explicitly enabled
  -> Ubuntu fixes the remotely allowed joint and hard caps
  -> Windows sends an authenticated, fresh arm request
  -> Ubuntu rejects any request outside its startup policy
  -> Ubuntu creates a bounded session
  -> existing signed steps use MoveIt and measured verification
```

Remote session arm is disabled by default. There is still no generic direct
execution endpoint.

## Safety Meaning

Starting the Ubuntu supervisor with `--enable-remote-session-arm` authorizes the
authenticated Windows computer to create new sessions until that supervisor is
stopped. It removes the per-session physical confirmation on Ubuntu.

Only enable it while all of these are true:

```text
the real workspace is clear
the Franka user stop is within reach
the Windows-Ubuntu cable is on the dedicated 10.10.10.x network
the shared secret is set only in the intended terminals
the configured joint and caps are appropriate for the current robot posture
```

Stopping the Ubuntu supervisor removes the remote arm endpoint and all future
remote session authority.

## Files

Ubuntu:

```text
scripts/fr3_bounded_session_supervisor.py
```

Transfer updated copies to `C:\vlm_bridge`:

```text
windows_vlm_bridge/fr3_action_protocol.py
windows_vlm_bridge/fr3_remote_session_control.py
```

## 1. Run Software Tests On Ubuntu

```bash
cd ~/franka_ros2_ws
python3 -m unittest -v \
  tests/test_fr3_action_protocol.py \
  tests/test_bounded_session_supervisor.py \
  tests/test_remote_session_control.py
```

These tests do not contact ROS, MoveIt, Windows, or the real robot.

## 2. Transfer The Windows Files

Ubuntu temporary file server:

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

Stop the temporary server with `Ctrl+C`.

Confirm the new protocol function exists:

```powershell
cd C:\vlm_bridge
python -c "import fr3_action_protocol as p; print(p.__file__); print(hasattr(p, 'create_session_arm_envelope'))"
```

Expected final value:

```text
True
```

## 3. Start MoveIt On Ubuntu

Ubuntu terminal 1:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2
```

Run only one MoveIt/`ros2_control` stack.

## 4. Start The Supervisor With A Narrow Remote Policy

For the first arm-and-stop test, use one joint, two actions, and `0.01 rad`
limits. Ubuntu terminal 2:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export FR3_BRIDGE_SECRET='YOUR_EXISTING_SHARED_SECRET'

python3 scripts/fr3_bounded_session_supervisor.py \
  --enable-bounded-session-execution \
  --enable-remote-session-arm \
  --remote-arm-joint fr3_joint3 \
  --bind-host 10.10.10.1 \
  --port 8080 \
  --allowed-client 10.10.10.2 \
  --session-max-steps 2 \
  --session-max-duration 60 \
  --session-max-cumulative-delta 0.02 \
  --max-delta 0.01 \
  --max-total-delta 0.01 \
  --max-moving-joints 1 \
  --velocity-scaling 0.03 \
  --acceleration-scaling 0.03
```

Required startup messages include:

```text
NO SESSION IS ARMED. THE ROBOT CANNOT MOVE FROM REMOTE REQUESTS YET.
Remote arm:  /v1/sessions/arm (fr3_joint3 only)
REMOTE SESSION ARM IS ENABLED UNTIL THIS SUPERVISOR STOPS.
Direct remote execution endpoint: NONE
```

The Ubuntu startup command is now the operator's authorization boundary. Do not
leave this supervisor running unattended.

## 5. Verify Health From Windows

```powershell
Invoke-RestMethod http://10.10.10.1:8080/health | ConvertTo-Json -Depth 6
```

Confirm:

```text
remote_session_arm_available: true
remote_arm_policy.allowed_joint: fr3_joint3
remote_arm_policy.max_steps: 2
remote_arm_policy.max_delta: 0.01
remote_arm_policy.max_cumulative_delta: 0.02
session_armed: false
```

## 6. Arm And Stop Without Robot Motion

Windows PowerShell:

```powershell
cd C:\vlm_bridge
conda activate vlm
$env:FR3_BRIDGE_SECRET = "YOUR_EXISTING_SHARED_SECRET"

python fr3_remote_session_control.py arm `
  --joint fr3_joint3 `
  --max-steps 2 `
  --max-duration 60 `
  --max-delta 0.01 `
  --max-cumulative-delta 0.02
```

Expected:

```text
REMOTE BOUNDED SESSION ARMED
Status:           ARMED
Allowed joint:    fr3_joint3
Completed steps:  0/2
```

The robot must not move. Arming creates bounded authority but submits no motion.

Inspect it from Windows:

```powershell
python fr3_remote_session_control.py status
```

Remove the authority:

```powershell
python fr3_remote_session_control.py stop
```

Expected status is `STOPPED`, with zero completed steps. Repeat this no-motion
test before allowing an execution script to use remotely created sessions.

## 7. Use The Existing Execution Client

After the arm-and-stop test passes, create a new Windows session with limits
that exactly match the Ubuntu startup caps. Then run the existing execution
client with `--run-armed-session`. It automatically discovers the fresh session,
just as it did for locally armed sessions.

The Qwen action must not exceed the new `0.01 rad` cap. If the current Qwen
server still maps observations to `0.03 rad`, keep this stage as arm-and-stop
only until the model-side action step is deliberately changed and retested.

## Rejection Behavior

Ubuntu rejects remote arm requests when:

- remote arm was not explicitly enabled at supervisor startup;
- the HMAC signature is missing or incorrect;
- the request is stale, future-dated, or replayed;
- the client IP is not allowed;
- the requested joint differs from `--remote-arm-joint`;
- any requested limit exceeds an Ubuntu startup cap; or
- another session is already active.

Windows can request smaller limits but can never increase Ubuntu's policy.

## Returning To Local-Only Arming

Stop the supervisor. Restart it without these arguments:

```text
--enable-remote-session-arm
--remote-arm-joint fr3_joint3
```

Health will report:

```text
remote_session_arm_available: false
session_arm_endpoint: null
```

Local `fr3_session_control.py arm` remains available through the private Ubuntu
socket.

No `colcon build` is needed. These are Python files.

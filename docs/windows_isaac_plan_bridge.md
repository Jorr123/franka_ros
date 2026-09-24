# Windows Isaac/Qwen to Ubuntu Plan-Only Bridge

This checkpoint changes the network direction to the one intended for Isaac:

```text
Windows static image (later: Isaac camera frame)
  -> Windows Qwen at 127.0.0.1:8000
  -> bounded action JSON
  -> authenticated HTTP request over 10.10.10.x
  -> Ubuntu FR3 AI supervisor
  -> Ubuntu validation
  -> MoveIt plan only
  -> plan result returned to Windows
```

The image and prompt stay on Windows. Only a small action envelope crosses to
Ubuntu. This supervisor version cannot execute a trajectory: it does not create
an ExecuteTrajectory client and exposes no execution endpoint.

## Files

Ubuntu:

```text
scripts/fr3_ai_supervisor.py
```

Transfer these files to `C:\vlm_bridge` on Windows:

```text
windows_vlm_bridge/isaac_qwen_coordinator.py
windows_vlm_bridge/fr3_action_protocol.py
```

The existing Windows Qwen server remains:

```text
windows_vlm_bridge/qwen_vlm_server.py
```

## 1. Transfer The Windows Files

On Ubuntu, temporarily serve the bridge directory:

```bash
cd ~/franka_ros2_ws/windows_vlm_bridge
python3 -m http.server 8081 --bind 10.10.10.1
```

On Windows PowerShell:

```powershell
Invoke-WebRequest `
  http://10.10.10.1:8081/isaac_qwen_coordinator.py `
  -OutFile C:\vlm_bridge\isaac_qwen_coordinator.py

Invoke-WebRequest `
  http://10.10.10.1:8081/fr3_action_protocol.py `
  -OutFile C:\vlm_bridge\fr3_action_protocol.py
```

Stop the temporary Ubuntu file server with `Ctrl+C`.

## 2. Configure One Shared Secret

Generate a temporary 32-byte secret in Windows PowerShell:

```powershell
python -c "import secrets; print(secrets.token_hex(32))"
```

Set the printed value in the Windows terminal that will run the coordinator:

```powershell
$env:FR3_BRIDGE_SECRET = "PASTE_THE_GENERATED_VALUE"
```

Set the identical value in the Ubuntu supervisor terminal:

```bash
export FR3_BRIDGE_SECRET='PASTE_THE_GENERATED_VALUE'
```

Do not put the secret in source code or an action JSON file. Both computer
clocks must be within two seconds because stale and future-dated requests are
rejected.

## 3. Start MoveIt On Ubuntu

Terminal 1:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2
```

Do not simultaneously launch a second `ros2_control_node`.

## 4. Start The Plan-Only Supervisor On Ubuntu

Terminal 2:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export FR3_BRIDGE_SECRET='PASTE_THE_GENERATED_VALUE'

python3 scripts/fr3_ai_supervisor.py \
  --bind-host 10.10.10.1 \
  --port 8080 \
  --allowed-client 10.10.10.2 \
  --min-confidence 0.70 \
  --max-delta 0.03 \
  --max-total-delta 0.03 \
  --max-moving-joints 1 \
  --goal-tolerance 0.002 \
  --velocity-scaling 0.03 \
  --acceleration-scaling 0.03
```

The final startup line must be:

```text
MODE: PLAN ONLY. This process cannot execute robot motion.
```

From Windows, check the health endpoint:

```powershell
Invoke-RestMethod http://10.10.10.1:8080/health
```

Confirm:

```text
mode: PLAN_ONLY
execution_available: False
```

## 5. Keep Qwen Running On Windows

Windows terminal 1:

```powershell
cd C:\vlm_bridge
conda activate vlm
python -m uvicorn qwen_vlm_server:app `
  --host 0.0.0.0 `
  --port 8000 `
  --workers 1
```

The coordinator calls this server through `127.0.0.1`; images no longer cross
from Ubuntu to Windows.

## 6. Windows Dry Run

Windows terminal 2:

```powershell
cd C:\vlm_bridge
conda activate vlm

python isaac_qwen_coordinator.py `
  --image C:\vlm_bridge\right.png `
  --min-confidence 0.70 `
  --max-delta 0.03 `
  --max-total-delta 0.03 `
  --max-moving-joints 1
```

Without `--send-plan`, Ubuntu is not contacted. Expected ending:

```text
DRY RUN complete. Ubuntu was not contacted. Add --send-plan to request a plan.
```

Inspect the printed action and envelope before continuing.

## 7. Send One Plan Request

Use the same command with `--send-plan`:

```powershell
python isaac_qwen_coordinator.py `
  --image C:\vlm_bridge\right.png `
  --min-confidence 0.70 `
  --max-delta 0.03 `
  --max-total-delta 0.03 `
  --max-moving-joints 1 `
  --send-plan
```

Expected Ubuntu response:

```text
status: PLAN_READY
plan_only: true
execution_available: false
planning.result: SUCCESS
```

The real robot must not move. Repeat with `left.png`. With `nothing.png`, the
coordinator should print `NO_ACTION` and must not contact Ubuntu.

## Rejection Behavior

Ubuntu rejects:

- missing or incorrect HMAC signatures;
- requests older than 10 seconds or too far in the future;
- reused request IDs;
- clients other than the configured Windows IP;
- bodies larger than 64 KiB;
- unknown joints, non-finite values, low confidence, zero motion, or excessive deltas;
- an observation whose left/right direction disagrees with the delta sign;
- targets too near or outside FR3 joint limits; and
- MoveIt planning failures.

Every coordinator invocation creates a new UUID and frame timestamp. A rejected
or lost request is not retried automatically.

## Checkpoint

This phase passes when all of the following are true:

```text
right.png   -> PLAN_READY with a positive delta; robot does not move
left.png    -> PLAN_READY with a negative delta; robot does not move
nothing.png -> NO_ACTION; Ubuntu is not contacted
duplicate request -> REPLAY_REJECTED
bad secret -> INVALID_SIGNATURE
```

After this checkpoint, test local Ubuntu supervised execution with the same
static images by following
[static_image_supervised_execution.md](static_image_supervised_execution.md).
The static image reader should be replaced by an Isaac camera-frame provider
only after that supervised checkpoint passes.

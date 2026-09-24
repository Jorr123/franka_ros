# Windows Qwen Vision Server

This server accepts the OpenAI-compatible image payload produced by:

```bash
python3 scripts/qwen_vlm_joint_action_test.py --backend openai-compatible --image IMAGE
```

It converts one base64 JPEG, PNG, or WebP image into Qwen2.5-VL input. Qwen
reports only whether a red marker is left, right, or unclear. Deterministic
server code maps left to `-0.01` rad and right to `+0.01` rad on the joint set
by `VISUAL_ACTION_JOINT`. Ubuntu remains the authoritative robot safety
validator.

The default is:

```python
VISUAL_ACTION_JOINT = "fr3_joint7"
VISUAL_ACTION_STEP = 0.01
MAX_ABS_ACTION_DELTA = 0.02
```

Change only `VISUAL_ACTION_JOINT` to select another FR3 arm joint. Restart the
Windows server after changing it.

## Transfer To Windows

Temporarily serve this directory from Ubuntu:

```bash
cd ~/franka_ros2_ws/windows_vlm_bridge
python3 -m http.server 8081 --bind 10.10.10.1
```

Download the files in Windows PowerShell:

```powershell
Invoke-WebRequest `
  http://10.10.10.1:8081/qwen_vlm_server.py `
  -OutFile C:\vlm_bridge\qwen_vlm_server.py

Invoke-WebRequest `
  http://10.10.10.1:8081/requirements.txt `
  -OutFile C:\vlm_bridge\requirements.txt
```

Stop the temporary Ubuntu file server with `Ctrl+C`.

## Start The Server

In Windows PowerShell:

```powershell
cd C:\vlm_bridge
conda activate vlm
python -m pip install -U qwen-vl-utils pillow
python -m uvicorn qwen_vlm_server:app `
  --host 0.0.0.0 `
  --port 8000 `
  --workers 1
```

Do not use `--reload` or more than one worker because each process may load a
copy of the model.

Verify from Ubuntu:

```bash
curl http://10.10.10.2:8000/health
```

The response should report `qwen-transformers-vision`, `max_images: 1`,
`visual_action_joint: fr3_joint7`, `visual_action_step: 0.01`, and
`decision_contract: red-marker-side-to-configured-joint`.

## Inference-Only Image Test

Use an image that is stored on Ubuntu. The client converts the file to a
base64 data URL before sending it to Windows:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 scripts/qwen_vlm_joint_action_test.py \
  --backend openai-compatible \
  --api-url http://10.10.10.2:8000/v1/chat/completions \
  --model-id Qwen/Qwen2.5-VL-7B-Instruct \
  --image /absolute/path/to/test-image.png \
  --task "Inspect the image and report whether the red marker center is left or right of the vertical dashed center line. Report unclear if it cannot be determined." \
  --action-space joint-deltas \
  --min-confidence 0.70 \
  --max-delta 0.02 \
  --max-total-delta 0.02 \
  --max-moving-joints 1 \
  --infer-only
```

The Windows server log must contain:

```text
[REQUEST] images=1
[VISION] processed_images=1
```

Do not use `--execute` during the first image tests.

## Windows-Originated Plan Requests

For the final network direction, keep the image and prompt on Windows and send
only the bounded action JSON to Ubuntu. Use:

```text
isaac_qwen_coordinator.py
fr3_action_protocol.py
```

The first Ubuntu supervisor is plan-only and cannot move the robot. Follow the
complete setup and test sequence in:

```text
docs/windows_isaac_plan_bridge.md
```

## Bounded Static-Image Shadow Session

After single-image planning works, transfer and run:

```text
static_vla_shadow_session.py
```

It processes an ordered image sequence with step, time, and cumulative-action
limits. It can call Qwen locally or request synchronous MoveIt plans, but it has
no execution option and refuses `--send-plans` unless Ubuntu reports
`PLAN_ONLY`. Follow:

```text
docs/static_vla_shadow_session.md
```

## Locally Armed Static-Image Execution Session

After the plan-only shadow session passes, use:

```text
static_vla_execution_session.py
```

Ubuntu must first run the separate bounded-session supervisor and the operator
must locally arm a session. The Windows coordinator can then submit only the
next signed sequence number and waits for measured `VERIFIED_SUCCESS` before
continuing. When no explicit `--session-id` is supplied, it discovers the one
fresh armed Ubuntu session automatically. There are no retries or generic
remote execution endpoint. Follow:

```text
docs/bounded_static_execution_session.md
```

## Optional Windows Session Arm

The bounded Ubuntu supervisor can optionally let Windows request creation of a
session. This capability is disabled by default. Ubuntu must explicitly enable
it at supervisor startup, preselect the remotely armable joint allowlist, and
set all moving-joint, step-total, time, per-joint, and cumulative caps.

Use:

```text
fr3_remote_session_control.py
```

to request, inspect, or stop the bounded session from Windows. Remote arming
does not move the robot; later steps still require the existing signed session
protocol, MoveIt execution, and measured verification. Follow:

```text
docs/bounded_multi_joint_session.md
```

The model-independent `bounded_joint_action_session.py` client accepts named
relative-joint JSON files. It is dry-run-only unless the explicit
`--run-armed-session` flag is present.

## Synthetic Visual Closed Loop

When no real camera is available, run:

```text
closed_loop_vla_shadow_session.py
```

It renders a synthetic red marker and creates every next frame from Qwen's
previous validated action. It runs locally by default, or `--send-plans` can
require a matching Ubuntu `PLAN_ONLY` response before each mock update. It has
no robot execution option. Follow:

```text
docs/mock_visual_closed_loop.md
```

## Synthetic Hardware-In-The-Loop Session

After right and left plan-only loops pass, use the separate:

```text
closed_loop_vla_execution_session.py
```

It requires one fresh, locally armed Ubuntu bounded session. Every synthetic
state update requires a measured `VERIFIED_SUCCESS` receipt from the FR3, and
there are no retries. Follow:

```text
docs/synthetic_hardware_loop.md
```

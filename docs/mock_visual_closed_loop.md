# Synthetic Visual Closed-Loop Shadow Test

This checkpoint tests observation-action feedback without a physical camera or
robot execution.

```text
render synthetic marker frame
  -> Qwen observes left/right/centered
  -> validate one bounded joint-3 action
  -> optionally require Ubuntu MoveIt PLAN_READY
  -> mock environment updates marker position
  -> render a new frame from the updated position
  -> repeat until Qwen returns NO_ACTION at center
```

The program is:

```text
windows_vlm_bridge/closed_loop_vla_shadow_session.py
```

It runs locally by default. With `--send-plans`, it sends authenticated planning
requests to Ubuntu and refuses to continue unless Ubuntu reports strict
`PLAN_ONLY` mode. The script has no execution option, so it cannot request robot
motion.

## Mock Physics

The default synthetic relationship is:

```text
positive fr3_joint3 delta -> marker moves left
negative fr3_joint3 delta -> marker moves right
0.03 rad                 -> 30 pixels
```

The marker begins 120 pixels from center and the center deadband is 30 pixels.
A correct policy therefore applies three `0.03 rad` mock actions and returns
no motion when the final error is 30 pixels or less.

This relationship exists only in the test harness. It does not establish how a
real camera image responds to FR3 joint motion.

## 1. Run The Tests On Ubuntu

```bash
cd ~/franka_ros2_ws
python3 -m unittest -v tests/test_closed_loop_vla_shadow_session.py
```

Expected:

```text
Ran 10 tests
OK
```

The tests include right and left convergence, perception and action failures,
limit enforcement, frame rendering, strict plan-before-update ordering, and
the rule that a failed plan must not update the mock environment.

## 2. Transfer The Updated Script To Windows

Ubuntu:

```bash
python3 -m http.server 8081 \
  --bind 10.10.10.1 \
  --directory ~/franka_ros2_ws/windows_vlm_bridge
```

Windows PowerShell:

```powershell
Invoke-WebRequest `
  "http://10.10.10.1:8081/closed_loop_vla_shadow_session.py?v=2" `
  -OutFile C:\vlm_bridge\closed_loop_vla_shadow_session.py
```

The script imports the existing current copies of:

```text
isaac_qwen_coordinator.py
static_vla_shadow_session.py
fr3_action_protocol.py
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

The tested Qwen mapping should be:

```text
right -> fr3_joint3 +0.03
left  -> fr3_joint3 -0.03
centered/unclear -> zero motion
```

## 4. Run The Local Synthetic Loop

Windows terminal 2:

```powershell
cd C:\vlm_bridge
conda activate vlm

python closed_loop_vla_shadow_session.py `
  --start-side right `
  --start-offset-px 120 `
  --deadband-px 30 `
  --joint fr3_joint3 `
  --max-delta 0.03 `
  --max-total-delta 0.03 `
  --max-actions 6 `
  --max-cumulative-delta 0.18 `
  --max-session-seconds 180 `
  --interval 0.5 `
  --output-dir C:\vlm_bridge\mock_runs\right_01
```

Expected progression:

```text
Observation 1: right,    x=440, error=+120 px -> +0.03
Observation 2: right,    x=410, error= +90 px -> +0.03
Observation 3: right,    x=380, error= +60 px -> +0.03
Observation 4: centered, x=350, error= +30 px -> NO_ACTION
```

Expected summary:

```text
Status:              GOAL_REACHED
Observations:        4
Actions applied:     3
MoveIt plans ready:  0
Cumulative delta:    0.090000 rad
Final marker side:   centered
Robot execution:     DISABLED (plan-only requests cannot execute)
```

Repeat with `--start-side left` and a new output directory such as
`C:\vlm_bridge\mock_runs\left_01`. The actions should be three `-0.03 rad`
deltas followed by centered `NO_ACTION`.

## 5. Add Strict Ubuntu Plan-Only Gating

This stage checks that each visually selected action is also accepted by the
real MoveIt planning stack. The robot should remain stationary.

Start the previously tested real-robot MoveIt launch in Ubuntu terminal 1:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2
```

Stop any bounded execution supervisor. Start the original supervisor in Ubuntu
terminal 2 without `--enable-local-execution`:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export FR3_BRIDGE_SECRET='YOUR_EXISTING_SHARED_SECRET'

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

Do not add any execution flag. Confirm this exact startup line before
continuing:

```text
MODE: PLAN ONLY. This process cannot execute robot motion.
```

Set the same secret and run the gated loop on Windows:

```powershell
$env:FR3_BRIDGE_SECRET = "YOUR_EXISTING_SHARED_SECRET"

python closed_loop_vla_shadow_session.py `
  --start-side right `
  --start-offset-px 120 `
  --deadband-px 30 `
  --joint fr3_joint3 `
  --max-delta 0.03 `
  --max-total-delta 0.03 `
  --max-actions 6 `
  --max-cumulative-delta 0.18 `
  --max-session-seconds 180 `
  --interval 0.5 `
  --output-dir C:\vlm_bridge\mock_runs\right_plan_01 `
  --send-plans
```

Every action must print `Ubuntu PLAN_READY` before the visual error changes.
The expected ending is:

```text
Status:              GOAL_REACHED
Actions applied:     3
MoveIt plans ready:  3
Robot execution:     DISABLED (plan-only requests cannot execute)
```

Repeat from the left with a new output directory. During this checkpoint the
physical robot joint positions should not change. Because the synthetic state
is independent of the real robot, each plan may begin from the same measured
FR3 state; this checkpoint validates the decision and planning contract, not
real visual dynamics.

## 6. Inspect Frames And Events

Each run directory contains sequential PNG frames and `events.jsonl`. Inspect
the log with:

```powershell
Get-Content C:\vlm_bridge\mock_runs\right_plan_01\events.jsonl
```

For each accepted action, the order must be:

```text
OBSERVATION_RENDERED
PLAN_READY
MOCK_ACTION_APPLIED
```

Each `after_error_px` must be smaller than `before_error_px`.

## Failure Statuses

`PREMATURE_NO_ACTION` means Qwen stopped outside the center deadband.

`PERCEPTION_MISMATCH` means Qwen's reported side disagreed with synthetic
ground truth.

`PLAN_FAILED` means Ubuntu rejected, timed out, or failed to plan the proposed
action. The mock environment is not updated.

`ACTION_DID_NOT_IMPROVE` means the action sign failed to reduce visual error.

`ACTION_LIMIT_REACHED` or `CUMULATIVE_LIMIT_REACHED` means a configured motion
budget was exhausted.

Any status other than `GOAL_REACHED` is a failed run. Do not increase limits to
hide a perception, planning, or direction failure.

## Checkpoint Result

Pass this checkpoint when local and plan-gated right/left runs repeatedly reach
center, Qwen returns no action at center, every action reduces visual error,
and each mock state update has exactly one preceding `PLAN_READY` event.

The next input source can be a prerecorded video sequence, then a webcam, and
finally an Isaac virtual camera. Real closed-loop execution requires measured
post-execution feedback and camera-to-robot calibration before it is enabled.

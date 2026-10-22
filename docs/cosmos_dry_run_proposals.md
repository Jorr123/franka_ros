# Cosmos to Ubuntu: validation-only action delivery

This stage sends the first Cosmos arm target to Ubuntu and receives an
acceptance/rejection receipt. It cannot move the arm or gripper.
The new receiver has no MoveIt client, ROS publisher, trajectory action client,
execution flag, or connection to the existing execution supervisor.

```text
Windows: cameras + measured state -> Cosmos -> first absolute joint target
                                            + fresh post-inference state
                                                        |
                                          validation-only proposal
                                                        v
Ubuntu : port 8083 -> check live ROS state + delta bounds -> receipt
```

## Files and transfer

The Ubuntu/shared code is in this Git repository:

- `robot_bridge/protocol.py`: model-independent proposal and receipt primitives.
- `robot_bridge/client.py`: one HTTP POST; no automatic retries.
- `robot_bridge/server.py`: validation receiver and audit log.
- `scripts/fr3_dry_run_action_server.py`: ROS subscriber and server entry point.
- `scripts/fr3_readonly_state_server.py`: the existing state-cache dependency.

Commit/push these changes from WSL, then pull that commit on Ubuntu using your
normal Git workflow. A plain pull cannot transfer uncommitted/unpushed files.
No ROS rebuild is needed for these Python scripts. Keep the existing read-only
state server running on port 8082.

On Windows, the sibling `inference/ubuntu_dry_run.py` connects Cosmos to this
package. It imports `franka_ros.franka_ros.robot_bridge` from the workspace root;
keep the current `vla_runtime/franka_ros/franka_ros/` checkout in that location.
No extra Python dependencies are required for the shared bridge.

## 1. Start the receiver on Ubuntu

Use the same ROS domain and middleware environment as the working state server.
The current robot and state launchers use ROS domain `42`; set it explicitly
in this new terminal too.
The current observed arm topic is `/franka/joint_states`. Substitute your
checkout path if it differs from `~/franka_ros2_ws`:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

python3 scripts/fr3_dry_run_action_server.py \
  --bind-host 10.10.10.1 \
  --allowed-client 10.10.10.2 \
  --joint-state-topic /franka/joint_states \
  --gripper-state-topic /franka_gripper/joint_states
```

No shared secret is required for this validation-only endpoint. The ROS domain
setting above is required to match the robot launchers. Expected startup:

```text
VALIDATION_ONLY: http://10.10.10.1:8083/v1/action-proposals/dry-run
Allowed clients: ['10.10.10.2'] | Robot execution: False
```

The receiver subscribes independently and reuses the existing strict state
cache: missing, stale, non-finite, or ambiguously sourced arm data is rejected.
Fresh measured gripper width is also required. It does not need another allowed
client added to the existing 8082 state endpoint. It binds only the named Ubuntu
interface and accepts the specified Windows IP.

## 2. Start Cosmos in WSL

Use the existing server if it is already running:

```bash
cd /home/jordan_vlm/vla_runtime
./run_cosmos_edge_policy.sh
```

## 3. Check the receiver and propose from Windows

In Windows PowerShell:

```powershell
$runtime = "\\wsl.localhost\Ubuntu-22.04\home\jordan_vlm\vla_runtime"
$python = Join-Path $env:LOCALAPPDATA "CosmosCameraBridge\.venv\Scripts\python.exe"

Invoke-RestMethod "http://10.10.10.1:8083/healthz"
```

Health should report `mode: VALIDATION_ONLY`, `execution_allowed: False`,
`robot_execution: False`, and `state_available: True`. If state is unavailable,
check ROS discovery, topics, publisher count, and gripper state on Ubuntu.

Keep the arm and gripper still, then run:

```powershell
& $python "$runtime\inference\run_cosmos3.py" propose-once `
  --gripper-open-width-m 0.08 `
  --prompt "pick up the object"
```

This captures live images/state, runs one new inference, fetches state again,
then sends only action index 0's seven absolute joint targets plus named deltas
computed as `target - fresh measured basis`. The target is preserved exactly;
it is not clipped or scaled. The remaining 31 targets and all predicted gripper
values are excluded. `gripper_mode: hold` describes exclusion; the receiver sends
no hold or other gripper command.

Expected receipt decisions:

- `ACCEPTED_FOR_DRY_RUN | VALIDATED`: the checks below passed; exit code 0.
- `REJECTED | <reason code>`: proposal delivered, matching receipt checked,
  but validation failed; exit code 2.
- A transport/receipt error: exit code 1; inspect `proposal_result.json`.
  No automatic retry is performed. A timeout may leave receipt status unknown.

Both acceptance and rejection are useful delivery tests. Acceptance is not
permission to execute and is not a collision, trajectory, or kinematic check.

## Checks and initial fixed limits

| Check | Limit |
| --- | --- |
| Per-joint delta, both declared and relative to Ubuntu's current joints | 0.01 rad |
| Total absolute delta (L1), both representations | 0.04 rad |
| Arm movement since inference observation | 0.005 rad per joint |
| Arm movement since fresh proposal basis | 0.002 rad per joint |
| Gripper movement since observation | 0.001 m |
| Live arm/gripper age and conservative basis age | 1 second |
| Observation age including capture, inference and submission | 30 seconds |
| Proposal age and time since inference completion | 5 seconds |
| Permitted future clock skew | 1 second |

The 30-second observation window accommodates the recorded roughly 18-second
inference for this stationary **dry-run test only**. It is not an execution
latency policy. Both computer clocks must agree within the stated skew. A new
proposal timestamp does not refresh the original observation time.

The receiver checks the publisher identity against its own live ROS discovery,
verifies complete finite named vectors and target-minus-basis consistency,
rejects repeated request UUIDs while they could remain valid, and requires
`mode: VALIDATION_ONLY`, `execution_allowed: false`, and `gripper_mode: hold`.
No secret or message signature is used in this validation-only service. Access
is restricted to the configured Windows client IP. Receipts are checked against
the request UUID and exact proposal hash; these checks correlate replies and do
not authenticate the sender. The client disables proxies and redirects and only
supports the dry-run path. The existing robot-execution bridge retains its
separate authentication.

The earlier recorded first Cosmos target differed by about 0.01369 rad on one
joint, so it would exceed these initial bounds. A similar rejection is expected;
do not loosen limits merely to obtain an acceptance.

## Artifacts and tests

Windows saves the normal inference files plus:

- `proposal.json`: the exact proposal, before HTTP submission.
- `proposal_receipt.json`: the matching receipt, when received and checked.
- `proposal_result.json`: delivery status or error; `RECEIPT_UNKNOWN` never means
  acceptance. `NOT_SENT` means proposal delivery was not attempted.

Ubuntu appends proposals and receipts to `log/dry_run_action_proposals.jsonl`.
Failure to save the audit record produces rejection. Logs contain robot state. This receiver has no execution counterpart to enable.

Offline receiver tests on Ubuntu/WSL, without ROS or hardware:

```bash
python3 -m unittest discover -s tests -p test_robot_bridge_dry_run.py -v
```

Windows regression and HTTP integration tests, with simulated cameras/state and
model responses:

```powershell
& $python -m unittest discover -s "$runtime\camera_bridge" -t "$runtime" -p "test_*.py" -v
& $python -m unittest discover -s "$runtime\inference" -t "$runtime" -p "test_*.py" -v
```

Physical execution remains separate work: validate DROID-to-FR3 conventions,
choose a trajectory/controller strategy, and test bounded motion with measured
feedback. Do not feed this dry-run message directly to the execution supervisor.

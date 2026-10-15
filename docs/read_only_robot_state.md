# Read measured FR3 state from Windows

`scripts/fr3_readonly_state_server.py` runs on the Ubuntu robot-control host.
It subscribes to `/franka/joint_states` and `/franka_gripper/joint_states` and answers
`GET /v1/robot-state` on the control network. It creates no motion publishers,
planning clients, or execution clients. Windows pulls a snapshot on each GET.

```text
Real FR3 -> existing Ubuntu ROS publishers -> read-only HTTP service
                                                    |
                                      Windows robot-state command
```

The defaults follow the existing lab addresses: Ubuntu `10.10.10.1`, Windows
`10.10.10.2`, and TCP port `8082`. Change the flags for your actual interfaces.
This endpoint restricts clients by IPv4 address; it provides neither encryption
nor authentication with a shared secret. Use it on the established control LAN.

## Transfer the new script

This script must exist in `scripts/` in the actual Ubuntu ROS workspace.
It can be transferred through Git after the change has been committed and
pushed. To copy this one file directly before that, use Windows PowerShell
(replace the username and destination workspace path as needed):

```powershell
$ubuntuUser = "YOUR_UBUNTU_USERNAME"
$workspace = "\\wsl.localhost\Ubuntu-22.04\home\jordan_vlm\vla_runtime"
scp "$workspace\franka_ros\franka_ros\scripts\fr3_readonly_state_server.py" "${ubuntuUser}@10.10.10.1:franka_ros2_ws/scripts/"
```

This assumes SSH is already available on Ubuntu and the destination directory
exists. The camera client stays in the Windows/WSL workspace.

## Start on Ubuntu

To start the robot stack and state server in two separate terminal windows,
first stop any separately running copies, then run from the Ubuntu desktop:

```bash
~/franka_ros2_ws/scripts/start_robot_and_state.sh
```

Both services use ROS domain `42`. The **FR3 Robot - MoveIt** window shows the
robot logs, and the **FR3 Read-Only State** window shows the server logs.
Press `Ctrl+C` in each window to stop that service. The two windows run
independently; stopping one does not stop the other. If a process exits
normally or fails, its window waits for Enter so you can read the output.
The HTTP endpoint may return 503 while the robot stack is starting or after
it stops. Extra arguments to this combined launcher are forwarded to the
robot launcher only.

To start the state server separately, use the following procedure.

After starting the robot with `scripts/start_robot.sh`, open a second Ubuntu
terminal and run:

```bash
~/franka_ros2_ws/scripts/start_read_state.sh
```

This launcher sources ROS and the workspace, sets `ROS_DOMAIN_ID=42`, and
starts the read-only server at `http://10.10.10.1:8082/v1/robot-state`.
It uses `/franka/joint_states` and allows the Windows client at `10.10.10.2`.
Leave the terminal open; press Ctrl+C to stop the server. Extra arguments are
forwarded to the Python server, for example `--port 8083` or `--help`.

The manual setup and source checks are shown below.

Use the established robot stack and a terminal with its ROS domain and DDS
configuration. The state service needs Python 3.10+, ROS 2 Humble `rclpy`, and
`sensor_msgs`. No Python package installation or `colcon build` is needed for
this standalone script.

The current `scripts/start_robot.sh` launches the robot in ROS domain `42`.
Use that domain in this terminal as well, and keep the other DDS settings the
same as the robot terminal. The HTTP connection from Windows is separate from
ROS discovery; Windows does not need to join the ROS domain for this endpoint.

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic info /franka/joint_states --verbose
ros2 topic echo /franka/joint_states --once
ros2 topic info /franka_gripper/joint_states --verbose
```

Confirm exactly one arm-state publisher and that it is the established real
robot source (normally `joint_state_broadcaster`). Names must include
`fr3_joint1` through `fr3_joint7`.

The current MoveIt launch remaps the hardware broadcaster to
`/franka/joint_states`. Its separate `joint_state_publisher` republishes state
on `/joint_states`; use the hardware topic so freshness checks are applied to
the original measured samples. Run the service in the same terminal:

```bash
python3 scripts/fr3_readonly_state_server.py \
  --bind-host 10.10.10.1 \
  --port 8082 \
  --allowed-client 10.10.10.2 \
  --joint-state-topic /franka/joint_states
```

The service only reads the already running ROS stack. Its startup does not
start the robot driver or the action supervisor. It refuses a wildcard bind
address such as `0.0.0.0`. Repeat `--allowed-client` to allow another specific
address. `--joint-state-topic` and `--gripper-state-topic` select namespaced
topics when needed. Press Ctrl+C to stop.

## Fetch on Windows

In Windows PowerShell, with WSL running, use the existing camera environment:

```powershell
$bridge = "\\wsl.localhost\Ubuntu-22.04\home\jordan_vlm\vla_runtime\camera_bridge"
$venv = Join-Path $env:LOCALAPPDATA "CosmosCameraBridge\.venv"
& "$venv\Scripts\python.exe" "$bridge\camera_bridge.py" robot-state
```

If that environment has not been created, `python "$bridge\camera_bridge.py"
robot-state` also works: the telemetry command uses only standard-library
Python. It can also run under WSL/Linux, provided Ubuntu allows the source IP
seen on that connection. Windows normally connects directly from `10.10.10.2`.

For repeated live readings:

```powershell
& "$venv\Scripts\python.exe" "$bridge\camera_bridge.py" robot-state --watch
```

The default interval is half a second. Change it with `--interval-seconds`;
`--json` emits one JSON object per snapshot. The sequence counts accepted ROS
samples, so it should increase even while the arm remains still. A failed or
invalid request stops the command, and Ctrl+C stops polling normally.

Neither cameras nor the Cosmos policy server need to run for this test.
`send-once` continues to be a dummy-state inference test. This transport does
not yet feed observations into Cosmos or convert policy outputs to FR3 actions.

## State checks and interpretation

- Joint positions are reordered by name into `fr3_joint1` through `fr3_joint7`
  and reported as `joint_positions_rad`.
- ROS header time is checked against Ubuntu's ROS clock on receipt. Zero,
  old, or more than 0.1 seconds future-dated stamps are rejected. Header age
  plus elapsed local time must remain within `--max-age-seconds` (default 1).
  Windows conservatively adds the entire HTTP round trip to that age before
  accepting it. Cross-computer clock synchronization is not required.
- Publisher discovery is refreshed every 0.25 seconds. The service requires
  exactly one visible publisher with an available endpoint identity. A
  publisher change invalidates the cached sample until a new sample arrives;
  stale discovery also blocks state. DDS discovery can take time to notice
  changes, and uniqueness alone does not prove a source is physical hardware.
  Check the publisher with the Ubuntu commands above. The JSON reports its
  node name, namespace, and endpoint GID for inspection.
- Missing/duplicate names, incomplete positions, non-finite values, and invalid
  timestamps immediately invalidate that channel rather than retaining the
  previous valid sample. An unavailable arm returns HTTP 503 with the reason.
- `gripper_width_m` is the sum of `fr3_finger_joint1` and `fr3_finger_joint2`.
  Missing, stale, or ambiguous gripper state is `null`, with an explanatory
  `gripper_unavailable_reason`. Arm and gripper samples have independent ages;
  they are not guaranteed to have been measured simultaneously.
- Raw gripper width has no validated conversion to the DROID policy's gripper
  input yet. Displaying live telemetry is the acceptance target here.

For a connection failure, check Ubuntu is listening on the expected address,
both computers' interface IPs, and whether the existing firewall permits TCP
8082 from the Windows IP. HTTP 403 means the source IP is outside the allowlist;
HTTP 503 reports a ROS source, message, or freshness problem. No firewall
configuration is changed by this script.

## Offline validation

From the Ubuntu checkout (also runnable without ROS):

```bash
python3 -m unittest discover -s tests -p test_fr3_readonly_state_server.py -v
```

From the Windows/WSL workspace root:

```bash
python3 -m unittest discover -s camera_bridge -p test_robot_state.py -v
```

These tests use fake state and localhost HTTP. Real ROS subscriptions and the
Windows-to-Ubuntu network must still be checked on the two machines.

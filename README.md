# FR3 ROS 2 Development Workspace

ROS 2 Humble workspace for a Franka Research 3 robot, with custom joint stream
controllers, MoveIt motion tools, keyboard control, and a Windows-to-Ubuntu
VLM/VLA action bridge.

The existing deployment runs robot control on native Ubuntu and inference on a
separate Windows computer. This repository makes the ROS and bridge source
available on both machines for development. The Cosmos/VLA setup on Windows is
a separate codebase and is **not included or verified as integrated here**.

## Start here

- **Publishing the existing workspace:** [GitHub preparation and push guide](docs/GITHUB_SETUP.md).
- **Cloning and building on Ubuntu or WSL:** [Development setup](docs/DEVELOPMENT.md).
- **Source origins and pinned versions:** [Upstream provenance](UPSTREAM.md).
- **Existing robot/controller commands:** [Controller operations](docs/CONTROLLER_OPERATIONS.md).

The inspected environment is Ubuntu 22.04.5 with ROS 2 Humble. The source
snapshot records the installed dependency revisions; a fresh Windows WSL build
has not been validated. Source access alone does not require a ROS installation.

## Start the robot stack

With the robot ready and FCI enabled in Franka Desk, run this in an Ubuntu terminal:

```bash
~/franka_ros2_ws/scripts/start_robot_and_state.sh
```

This opens two separate terminal windows on the Ubuntu desktop: **FR3 Robot -
MoveIt** and **FR3 Read-Only State**. Stop any separately running copies first.
Each window shows its own logs; press `Ctrl+C` in each window to stop its service.
The windows run independently. The state endpoint can report unavailable while
the robot stack is starting or after the robot stack stops.

To start the robot stack on its own:

```bash
~/franka_ros2_ws/scripts/start_robot.sh
```

The launcher sources ROS 2 Humble and this workspace, sets `ROS_DOMAIN_ID=42`,
and starts MoveIt for the robot at `172.16.0.2`. Leave the terminal open; press
`Ctrl+C` to stop the launch. Use `ROS_DOMAIN_ID=42` in other ROS terminals too.

To serve measured robot state to Windows, open a second terminal and run:

```bash
~/franka_ros2_ws/scripts/start_read_state.sh
```

This launcher also sources ROS and sets domain `42`. See the
[state endpoint guide](docs/read_only_robot_state.md) for Windows access.

## Repository layout

| Path | Contents |
| --- | --- |
| `src/` | Modified Franka ROS 2 packages, custom controllers, launch files, MoveIt configurations, and upstream notices. |
| `scripts/` | Ubuntu motion helpers, keyboard tools, supervisors, diagnostics, and source export helper. |
| `windows_vlm_bridge/` | Qwen server example, action protocol, Windows session clients, and sample actions. |
| `robot_bridge/` | Model-independent, IP-restricted validation-only proposals and receipts; no execution backend. |
| `config/` | Fast DDS network profile for the original Ubuntu host. |
| `tests/` | Python unit tests and configuration checks. |
| `docs/` | Setup instructions, technical operation guides, and historical handoffs. |
| `dependencies.repos` | Exact external dependency commits to restore after cloning. |

`build/`, `install/`, `log/`, recordings, virtual environments, and downloaded
source dependencies are excluded. See the [inclusion table](docs/GITHUB_SETUP.md#what-is-included)
for the publication decisions.

## Development and runtime architecture

```text
Windows: model runtime / action adapter
                 |
          authenticated action proposal
                 v
Ubuntu: supervisor -> validation -> MoveIt -> ros2_control -> FR3
                 ^                                      |
                 +------- measured-state checks --------+
```

The bridge supports plan-only requests and separately enabled bounded execution
sessions. The proposed model adapter must match the existing action contract.
Reading both repositories does not start inference or connect to the robot.

Keep physical robot execution on the established Ubuntu host during Windows
integration work. Source-only review, unit tests, and fake hardware results do
not establish that a new setup is ready to control physical hardware.

## Technical guides

| Topic | Guide |
| --- | --- |
| Read-only FR3 state for Windows | [Robot state endpoint](docs/read_only_robot_state.md) |
| Cosmos proposals without execution | [Validation-only delivery](docs/cosmos_dry_run_proposals.md) |
| Custom Isaac joint stream controller | [Controller explanation](docs/isaac_joint_stream_controller_explained.md) |
| Windows plan-only bridge | [Plan-only integration](docs/windows_isaac_plan_bridge.md) |
| Bounded joint action sessions | [Multi-joint session contract](docs/bounded_multi_joint_session.md) |
| Keyboard joint jogging | [Discrete joint jog](docs/discrete_keyboard_joint_jog.md) |
| Cartesian motion | [Relative Cartesian control](docs/relative_cartesian_control.md) |
| Keyboard Cartesian Servo | [Continuous Cartesian Servo](docs/fr3_continuous_cartesian_servo.md) |
| Existing Windows Qwen server | [Bridge README](windows_vlm_bridge/README.md) |

Older documents mentioning Qwen, GR00T, and SmolVLA describe earlier development
checkpoints. Their model choices, machine paths, and reported results should be
read in that historical context.

## Source attribution

The Franka ROS 2 base and local modifications are recorded in
[UPSTREAM.md](UPSTREAM.md). Existing upstream [license](src/LICENSE) and
[notice](src/NOTICE) files are retained. Repository preparation has not assigned
a new blanket license to the workspace's original scripts and documentation.

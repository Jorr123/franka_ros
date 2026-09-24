# Development on Ubuntu and Windows WSL

The inspected host runs Ubuntu 22.04.5 LTS with ROS 2 Humble. Match Ubuntu 22.04
and ROS 2 Humble in WSL when reproducing this workspace. Python 3.10 is the
Ubuntu 22.04 baseline; the project uses modern type annotations.

Cloning the source is sufficient for code review. The steps below are for
building and running ROS-dependent development tools.

## Computer roles

| Computer | Role |
| --- | --- |
| Existing native Ubuntu host | ROS 2, MoveIt, supervisors, `ros2_control`, and communication with the physical FR3. |
| Windows machine | Existing Cosmos/VLA runtime and any Windows-side inference or Isaac Sim work. |
| Ubuntu under WSL | A second source checkout for editing, tests, and development builds; hardware behavior must be validated separately. |

Copying source does not copy the installed ROS environment, GPU runtime,
system configuration, or active processes. This repository's Windows bridge
contains Qwen and action-session examples. The separate Cosmos setup has not
been inspected or imported; its compatibility with this bridge is not yet
established.

The upstream [README](../src/README.md) says native Windows is not officially
supported for `franka_ros2`. WSL build instructions here have not been tested on
your Windows machine. Keep physical robot control on the existing Ubuntu host
while developing on Windows.

## Prepare Ubuntu 22.04

Install ROS 2 Humble using the
[official Ubuntu installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
After configuring its apt repository and installing ROS:

```bash
sudo apt update
sudo apt install git python3-vcstool python3-rosdep \
  python3-colcon-common-extensions python3-pil python3-yaml \
  ros-humble-pinocchio
source /opt/ros/humble/setup.bash
```

On a new machine where rosdep has never been initialized, run `sudo rosdep init`
once. Then run:

```bash
rosdep update
```

## Restore external source dependencies

Run in a fresh clone of the exported repository:

```bash
cd ~/franka_ros2_ws
vcs import src < dependencies.repos
git -C src/libfranka submodule update --init --recursive
rosdep install --from-paths src --ignore-src --rosdistro humble -y
```

The manifest restores exact commits of `libfranka` and `franka_description`.
The submodule command restores `libfranka/common`. Do not reclone upstream
`franka_ros2` over `src/`: this repository already contains the modified source.
For an existing dependency checkout, inspect its changes before switching
versions. See [UPSTREAM.md](../UPSTREAM.md) for the recorded revisions.

## Build

In a new terminal, source the ROS underlay and build the complete workspace:

```bash
cd ~/franka_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The older controller guide's `--packages-select` command is for incremental
builds of an already prepared workspace. Use the full build for a fresh clone.
The build recreates `build/`, `install/`, and `log/`; they remain ignored by Git.

## Python tests

After installing ROS dependencies, run the workspace tests from its root:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m unittest discover -s tests -v
```

Some tests import ROS message packages and `rclpy`, while protocol tests use
fake planners and local HTTP servers. This is not a standard-library-only test
suite and is not a physical robot acceptance test. Pillow and PyYAML are also
used. Start with the checked-in tests before attempting any hardware session.

## Machine-specific configuration

Existing guides record these lab values:

| Setting | Existing example |
| --- | --- |
| Ubuntu control-network interface | `10.10.10.1` |
| Windows control-network interface | `10.10.10.2` |
| FR3 robot address | `172.16.0.2` |
| ROS domain used in examples | `77` |

`config/fastdds_control.xml` binds Fast DDS to loopback and `10.10.10.1`.
That is an Ubuntu-host profile, not a portable WSL networking configuration.
Choose interface addresses and firewall settings for the actual machine before
using DDS across hosts. The HTTP bridge and DDS communication are separate
connections; sharing a `ROS_DOMAIN_ID` alone does not configure networking.

Bridge authentication uses `FR3_BRIDGE_SECRET` in each process environment.
Set matching local values when using the authenticated bridge. Preserve its
timestamp, signature, validation, and session checks when adapting another
model's output.

Historical guides contain `/home/itri`, `~/franka_ros2_ws`, and `C:\vlm_bridge`
paths. Substitute your checkout paths. The Windows Qwen requirements are in
`windows_vlm_bridge/requirements.txt`; install them only into a separate
environment if using that Qwen server. They do not describe the Cosmos runtime.

## Working with both codebases

Keep this repository and the Cosmos source as separate folders/repositories
available to your development session. Document which Cosmos entry point runs
on Windows and how its output maps to the action contract before implementing
an adapter. Git transfers code changes; testing Windows inference still needs
the Windows runtime and testing FR3 hardware still needs the Ubuntu host.

Start with [the plan-only bridge guide](windows_isaac_plan_bridge.md) and
[the multi-joint action contract](bounded_multi_joint_session.md). Older handoff
documents describe earlier Qwen, GR00T, or SmolVLA experiments and are historical
context, not evidence that Cosmos has been integrated.

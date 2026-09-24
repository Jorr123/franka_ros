# Source provenance and dependencies

This workspace includes a modified working copy of Franka Robotics' ROS 2
integration plus project-specific Python scripts, tests, configurations, and
Windows bridge code. The publication export is a source snapshot; it does not
include the original repositories' Git history.

## Franka ROS 2 source

- Upstream: <https://github.com/frankarobotics/franka_ros2>
- Source location: `src/`
- Local branch at preparation: `humble`
- Base commit: `7ef0ab00964a0d2b5e19e9292bbcef9f32dda811`
- Local changes are included in the exported source even when uncommitted.

Modified upstream files at preparation:

- `src/franka_bringup/config/controllers.yaml`
- `src/franka_bringup/config/franka.config.yaml`
- `src/franka_example_controllers/CMakeLists.txt`
- `src/franka_example_controllers/franka_example_controllers.xml`
- `src/franka_fr3_moveit_config/config/fr3_ros_controllers.yaml`
- `src/franka_fr3_moveit_config/config/ompl_planning.yaml`
- `src/franka_fr3_moveit_config/launch/moveit.launch.py`
- `src/franka_fr3_moveit_config/package.xml`

Additional source includes two Isaac joint stream controllers and their headers,
MoveIt Servo configuration and launch code, joint limits, Pilz planner
configuration, and joint 7 tuning profiles. Workspace-level `scripts/`, `tests/`,
`config/`, and `windows_vlm_bridge/` contain the integration work.

## Dependencies restored after cloning

Both dependency checkouts were clean when inspected. They are excluded from the
publication snapshot and restored using [dependencies.repos](dependencies.repos).

| Dependency | Observed version | Exact commit |
| --- | --- | --- |
| [franka_description](https://github.com/frankarobotics/franka_description) | 1.3.0 | `1ccde30d5a30a710f335c9f6545528447a04bc7c` |
| [libfranka](https://github.com/frankarobotics/libfranka) | 0.19.0 | `4df37dd2b527f41ef34b76d6d4d3805864b09993` |

`libfranka` also requires its `common` Git submodule. Its observed commit is
`d96c221a32c343384cbe2f77c61116ed58b84231`, from
<https://github.com/frankarobotics/libfranka-common>. Initializing submodules
after restoring `libfranka` selects the revision recorded by that checkout.

The original `src/dependency.repos` uses version tags. The root manifest records
the exact commits from this installation and is used by the new setup guide.
It pins source dependencies only; it does not lock all apt or Python packages.

## Existing license and notice files

The upstream Franka ROS 2 source carries [src/LICENSE](src/LICENSE) and
[src/NOTICE](src/NOTICE). The Gazebo integration also retains its
[license file](src/franka_gazebo/franka_ign_ros2_control/LICENSE).
Keep these files and existing source notices with the copied source.

No new blanket license has been assigned to the original workspace scripts or
documentation as part of repository preparation. Restored dependencies retain
their own license files. See those files for their terms.

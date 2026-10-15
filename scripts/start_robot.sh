#!/usr/bin/env bash
# Start the FR3 MoveIt stack from this workspace.
set -eo pipefail

FR3_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$FR3_WORKSPACE"

for setup_file in /opt/ros/humble/setup.bash "$FR3_WORKSPACE/install/setup.bash"; do
    if [[ ! -r "$setup_file" ]]; then
        printf 'Missing ROS setup file: %s\n' "$setup_file" >&2
        exit 1
    fi
done

source /opt/ros/humble/setup.bash
source "$FR3_WORKSPACE/install/setup.bash"
export ROS_DOMAIN_ID=42

exec ros2 launch franka_fr3_moveit_config moveit.launch.py \
    robot_ip:=172.16.0.2 "$@"

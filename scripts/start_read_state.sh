#!/usr/bin/env bash
# Serve measured FR3 state using the same ROS domain as start_robot.sh.
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
export ROS_DOMAIN_ID=99

exec python3 "$FR3_WORKSPACE/scripts/fr3_readonly_state_server.py" "$@"

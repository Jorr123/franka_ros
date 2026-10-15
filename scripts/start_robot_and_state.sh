#!/usr/bin/env bash
# Open separate Ubuntu terminal windows for the FR3 stack and state server.
set -eo pipefail

FR3_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    printf 'Usage: %s [ROS launch arguments...]\n' "$0"
    printf 'Opens start_robot.sh and start_read_state.sh in separate terminal windows.\n'
    printf 'Press Ctrl+C in each service window to stop that service.\n'
    printf 'Extra arguments are forwarded to start_robot.sh only.\n'
    exit 0
fi

if ! command -v gnome-terminal >/dev/null; then
    printf 'Missing gnome-terminal. Run this launcher on the Ubuntu desktop.\n' >&2
    exit 1
fi
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    printf 'No desktop display found. Run this script from an Ubuntu desktop terminal.\n' >&2
    exit 1
fi
for launcher in start_robot.sh start_read_state.sh; do
    if [[ ! -x "$FR3_WORKSPACE/scripts/$launcher" ]]; then
        printf 'Missing executable launcher: %s/scripts/%s\n' "$FR3_WORKSPACE" "$launcher" >&2
        exit 1
    fi
done

open_service_window() {
    local title="$1" launcher="$2"
    shift 2
    gnome-terminal --window --title="$title" --working-directory="$FR3_WORKSPACE" \
        -- bash -c '
            "$@"
            status=$?
            printf "\nProcess exited with status %s.\n" "$status"
            read -r -p "Press Enter to close this window... "
            exit "$status"
        ' bash "$launcher" "$@"
}

open_service_window 'FR3 Robot - MoveIt' "$FR3_WORKSPACE/scripts/start_robot.sh" "$@"
open_service_window 'FR3 Read-Only State' "$FR3_WORKSPACE/scripts/start_read_state.sh"
printf 'Opened robot and state server in two separate terminal windows.\n'
printf 'Press Ctrl+C in each service window to stop it.\n'

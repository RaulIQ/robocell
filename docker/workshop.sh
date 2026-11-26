#!/usr/bin/env bash
set -eo pipefail
mode=${ROBOCELL_MODE:-workshop}
case "$mode" in
    workshop|mapping|follow) ;;
    cube) mode=follow; set -- fake_target:=true "$@" ;;
    *)
        printf 'Invalid ROBOCELL_MODE "%s", expected workshop, mapping, follow or cube\n' "$mode" >&2
        exit 2
        ;;
esac
source_scan=${ROBOCELL_SCAN_SOURCE:-lidar}
case "$source_scan" in
    lidar|depth) ;;
    *)
        printf 'Invalid ROBOCELL_SCAN_SOURCE "%s", expected lidar or depth\n' "$source_scan" >&2
        exit 2
        ;;
esac
localize=${ROBOCELL_LOCALIZE:-false}
database=${ROBOCELL_RTABMAP_DATABASE:-/ws/log/rtabmap.db}
case "$localize" in
    false) ;;
    true)
        if [[ "$source_scan" != depth || "$mode" == workshop ]]; then
            printf 'ROBOCELL_LOCALIZE=true requires depth scans and mapping, follow or cube mode\n' >&2
            exit 2
        fi
        if [[ ! -s "$database" ]]; then
            printf 'RTAB-Map database %s is missing or empty\n' "$database" >&2
            exit 2
        fi
        ;;
    *)
        printf 'Invalid ROBOCELL_LOCALIZE "%s", expected true or false\n' "$localize" >&2
        exit 2
        ;;
esac
if [[ "$source_scan" == depth && "$mode" != workshop ]]; then
    mkdir -p -- "$(dirname -- "$database")"
    set -- "localize:=$localize" "rtabmap_database:=$database" "$@"
fi
cd /ws
source /opt/ros/jazzy/setup.bash
python -m colcon build --symlink-install --packages-up-to robocell_sim
source install/setup.bash
exec ros2 launch robocell_sim "$mode.launch.xml" "scan_source:=$source_scan" "$@"

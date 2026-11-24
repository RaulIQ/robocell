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
cd /ws
source /opt/ros/jazzy/setup.bash
python -m colcon build --symlink-install --packages-up-to robocell_sim
source install/setup.bash
exec ros2 launch robocell_sim "$mode.launch.xml" "$@"

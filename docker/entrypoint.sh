#!/usr/bin/env bash
# Sources ROS and the built workspace, then runs the command.
set -e
source "/opt/ros/${ROS_DISTRO:?}/setup.bash"
if [ -f /cache/install/setup.bash ]; then
    source /cache/install/setup.bash
fi
exec "$@"

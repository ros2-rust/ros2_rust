#!/bin/bash
# Build the interface packages of the sourced ROS 2 distribution with the
# pinned rosidl_rust, like CI does. Source <overlay-dir>/install/setup.sh
# afterwards so that ros-env uses these bindings.
#
# Usage: overlay.sh [--rosdep] <overlay-dir>
#
#   --rosdep  install the dependencies with rosdep first (needs root or sudo)
#
# To update an existing overlay, remove it and run the script again.

set -euo pipefail

usage() { echo "usage: overlay.sh [--rosdep] <overlay-dir>" >&2; exit 1; }

ROSDEP=0
if [ "${1:-}" = --rosdep ]; then
    ROSDEP=1
    shift
fi
[ $# -eq 1 ] || usage
OVERLAY=$1

[ -n "${ROS_DISTRO:-}" ] || { echo "error: ROS_DISTRO is not set, source a ROS 2 installation first" >&2; exit 1; }
REPOS=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
[ -f "$REPOS/ros2_rust_$ROS_DISTRO.repos" ] || { echo "error: there is no $REPOS/ros2_rust_$ROS_DISTRO.repos" >&2; exit 1; }

mkdir -p "$OVERLAY/src"
cd "$OVERLAY"
vcs import src < "$REPOS/ros2_rust_$ROS_DISTRO.repos"
vcs import src < "$REPOS/ros2_rust_generator.repos"

if [ "$ROSDEP" = 1 ]; then
    rosdep install -y --rosdistro "$ROS_DISTRO" --ignore-src --from-paths src
fi

colcon build

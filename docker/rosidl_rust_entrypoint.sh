#!/bin/bash
set -e

# The overlay sources the ROS 2 installation as well.
source /opt/ros2_rust_overlay/install/setup.bash --

exec "$@"

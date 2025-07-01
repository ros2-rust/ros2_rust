#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --

# If we're a distro that needs the overlay built, then source it
if [ -f "/tmp/rosidl_rust_overlay/install/setup.bash" ]; then
  source /tmp/rosidl_rust_overlay/install/setup.bash --
fi

exec "$@"

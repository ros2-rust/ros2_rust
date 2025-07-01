#!/bin/bash

# Depending on the ROS_DISTRO, make sure we've got rosidl_generator_rs
# sourced and available in working directory workspace

if [ "$ROS_DISTRO" = "rolling" ]; then
  apt-get update && apt-get install -y ros-$ROS_DISTRO-rosidl-generator-rs
else
  # Temporarily add `rosidl_rust` to an overlay, build, and source it.
  mkdir -p /tmp/rosidl_rust_overlay/src
  git clone https://github.com/ros2-rust/rosidl_rust /tmp/rosidl_rust_overlay/src

  cd /tmp/rosidl_rust_overlay

  . /opt/ros/$ROS_DISTRO/setup.sh
  colcon build
fi

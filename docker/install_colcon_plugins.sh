#!/bin/bash

if [ "$ROS_DISTRO" = "humble" ]; then
  pip install --upgrade pytest
  pip install colcon-ros-cargo
else
  pip install --break-system-packages pytest colcon-ros-cargo
fi

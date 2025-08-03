#!/bin/bash

if [ "$ROS_DISTRO" = "humble" ]; then
  pip install --upgrade pytest
  pip install \
    git+https://github.com/colcon/colcon-cargo.git \
     git+https://github.com/colcon/colcon-ros-cargo.git
else
  pip install --break-system-packages pytest \
    git+https://github.com/colcon/colcon-cargo.git \
    git+https://github.com/colcon/colcon-ros-cargo.git
fi

ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO AS base
ARG DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    curl \
    git \
    libclang-dev \
    tmux \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Rust
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- --default-toolchain 1.85.0 -y
ENV PATH=/root/.cargo/bin:$PATH

COPY src/ros2_rust/docker/install_colcon_plugins.sh /
RUN ./install_colcon_plugins.sh

# The interface packages, built with the pinned generator like in CI.
COPY src/ros2_rust/docker/overlay.sh /ros2_rust/docker/
COPY src/ros2_rust/ros2_rust_*.repos /ros2_rust/
RUN apt-get update && rosdep update --rosdistro $ROS_DISTRO \
    && . /opt/ros/$ROS_DISTRO/setup.sh \
    && /ros2_rust/docker/overlay.sh --rosdep /opt/ros2_rust_overlay \
    && rm -rf /var/lib/apt/lists/* /opt/ros2_rust_overlay/build /opt/ros2_rust_overlay/log

RUN mkdir -p /workspace && echo "Did you forget to mount the repository into the Docker container?" > /workspace/HELLO.txt
WORKDIR /workspace

COPY src/ros2_rust/docker/rosidl_rust_entrypoint.sh /
ENTRYPOINT ["/rosidl_rust_entrypoint.sh"]
CMD ["/bin/bash"]

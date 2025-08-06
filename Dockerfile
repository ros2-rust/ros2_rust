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
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- --default-toolchain 1.75.0 -y
ENV PATH=/root/.cargo/bin:$PATH

COPY src/ros2_rust/docker/install_colcon_plugins.sh /
RUN ./install_colcon_plugins.sh

COPY src/ros2_rust/docker/rosidl_rust_setup.sh /
RUN ./rosidl_rust_setup.sh

RUN mkdir -p /workspace && echo "Did you forget to mount the repository into the Docker container?" > /workspace/HELLO.txt
WORKDIR /workspace

COPY src/ros2_rust/docker/rosidl_rust_entrypoint.sh /
ENTRYPOINT ["/rosidl_rust_entrypoint.sh"]
CMD ["/bin/bash"]

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

# Install the colcon-cargo and colcon-ros-cargo plugins
RUN if [ "$ROS_DISTRO" = "humble" ] ;  \
    then pip install --upgrade pytest && pip install colcon-ros-cargo ;  \
    else pip install --break-system-packages pytest colcon-ros-cargo ; fi

RUN mkdir -p /workspace && echo "Did you forget to mount the repository into the Docker container?" > /workspace/HELLO.txt
WORKDIR /workspace

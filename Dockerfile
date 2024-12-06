ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO AS base
ARG DEBIAN_FRONTEND=noninteractive

ARG CONTAINER_USER="cuser"
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a user in the container to operate as unprivileged
RUN groupadd --gid $USER_GID $CONTAINER_USER \
    && useradd --uid $USER_UID --gid $USER_GID -m $CONTAINER_USER \
    && mkdir -p /home/$CONTAINER_USER
RUN apt-get update \
    && apt-get install -y sudo \
    && rm -rf /var/lib/apt/lists/* \
    && echo $CONTAINER_USER ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$CONTAINER_USER \
    && chmod 0440 /etc/sudoers.d/$CONTAINER_USER

SHELL ["/bin/bash", "--login", "-c"]

# Install dependencies
RUN apt-get update && apt-get install -y \
    curl \
    git \
    libclang-dev \
    tmux \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*


# Install Rust
USER $CONTAINER_USER
WORKDIR /home/${CONTAINER_USER}
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- --default-toolchain 1.75.0 -y
ENV PATH=/root/.cargo/bin:$PATH


RUN pip install --upgrade pytest 

# Install the colcon-cargo and colcon-ros-cargo plugins
RUN pip install git+https://github.com/colcon/colcon-cargo.git git+https://github.com/colcon/colcon-ros-cargo.git

RUN mkdir -p ~/workspace && echo "Did you forget to mount the repository into the Docker container?" > ~/workspace/HELLO.txt
WORKDIR /home/${CONTAINER_USER}/workspace

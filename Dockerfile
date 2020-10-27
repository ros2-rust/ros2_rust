# This dockerfile can be configured via --build-arg
# Build context must be the /ros2_rust root folder for COPY.
# Example build command:
# export OVERLAY_MIXINS="debug ccache coverage"
# export RUN_TESTS="true"
# docker build -t nav2:latest \
#   --build-arg OVERLAY_MIXINS \
#   --build-arg RUN_TESTS 
#   --pull ./

ARG FROM_IMAGE=ros:foxy
ARG OVERLAY_WS=/opt/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher

# clone overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
COPY ./ros2_rust.repos ../
RUN vcs import ./ < ../ros2_rust.repos && \
    find ./ -name ".git" | xargs rm -rf
COPY ./ ./ros2-rust/ros2_rust

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt && \
    find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp/opt || true

# multi-stage for building
FROM $FROM_IMAGE AS builder
ARG DEBIAN_FRONTEND=noninteractive

# install CI dependencies
RUN apt-get update && apt-get install -q -y \
        ccache \
        clang \
        lcov \
        libclang-dev \
        llvm-dev \
        wget \
    && rm -rf /var/lib/apt/lists/*

# install rust dependencies
ENV RUSTUP_HOME=/usr/local/rustup \
    CARGO_HOME=/usr/local/cargo \
    PATH=/usr/local/cargo/bin:$PATH \
    RUST_VERSION=1.47.0
RUN set -eux; \
    wget -O rustup-init "https://sh.rustup.rs"; \
    chmod +x rustup-init; \
    ./rustup-init -y \
      --no-modify-path \
      --default-toolchain $RUST_VERSION; \
    rm rustup-init; \
    chmod -R a+w $RUSTUP_HOME $CARGO_HOME; \
    rustup --version; \
    cargo --version; \
    rustc --version;

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS ./
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build overlay source
COPY --from=cacher $OVERLAY_WS ./
ARG OVERLAY_MIXINS="release ccache"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin $OVERLAY_MIXINS \
    || ([ -z "$FAIL_ON_BUILD_FAILURE" ] || exit 1)

# source overlay from entrypoint
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place \
      's|^source .*|source "$OVERLAY_WS/install/setup.bash"|' \
      /ros_entrypoint.sh

# test overlay build
ARG RUN_TESTS
ARG FAIL_ON_TEST_FAILURE=True
RUN if [ -n "$RUN_TESTS" ]; then \
        . install/setup.sh && \
        colcon test && \
        colcon test-result \
          || ([ -z "$FAIL_ON_TEST_FAILURE" ] || exit 1) \
    fi

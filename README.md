ROS 2 for Rust
==============

| Target | Status |
|----------|--------|
| **Ubuntu 20.04** | [![Build Status](https://github.com/ros2-rust/ros2_rust/actions/workflows/rust.yml/badge.svg?branch=master)](https://github.com/ros2-rust/ros2_rust/actions/workflows/rust.yml?branch=master) |

Introduction
------------

This is a set of projects (the `rclrs` client library, code generator, examples and more) that
enables developers to write ROS 2 applications in Rust.

Features and limitations
------------------------

The current set of features include:
- Message generation
- Support for publishers and subscriptions
- Tunable QoS settings

Lots of things are still missing however, see the [issue list](https://github.com/ros2-rust/ros2_rust/issues) for an overview.

The client library is still rapidly evolving, and there are no stability guarantees.

Sounds great, how can I try this out?
-------------------------------------

In a nutshell, the steps to get started are:

```shell
mkdir -p workspace/src && cd workspace
git clone https://github.com/ros2-rust/ros2_rust.git src/ros2_rust
docker build -t ros2_rust_dev - < src/ros2_rust/Dockerfile
docker run --rm -it --volume $(pwd):/workspace ros2_rust_dev /bin/bash
# The following steps are executed in Docker
vcs import src < src/ros2_rust/ros2_rust_foxy.repos
tmux
colcon build
```

Then, to run the minimal pub-sub example, do this:

```shell
# In a new terminal (tmux window) inside Docker
. ./install/setup.sh
ros2 run rclrs_examples minimal_publisher
# In a new terminal (tmux window) inside Docker
. ./install/setup.sh
ros2 run examples_rclrs_minimal_pub_sub minimal_subscriber
```

For an actual guide, see the following documents:
- [Building `ros2_rust` packages](docs/Building.md)
- [Contributing to `ros2_rust`](docs/Contributing.md)

Let us know if you build something cool with `ros2_rust`!
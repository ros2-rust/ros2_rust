ROS 2 for Rust
==============

| Target | Status |
|----------|--------|
| **Ubuntu 20.04** | [![Build Status](https://github.com/ros2-rust/ros2_rust/actions/workflows/rust.yml/badge.svg?branch=main)](https://github.com/ros2-rust/ros2_rust/actions/workflows/rust.yml?branch=main) |

Introduction
------------

This is a set of projects (the `rclrs` client library, code generator, examples and more) that
enables developers to write ROS 2 applications in Rust.

Features and limitations
------------------------

The current set of features include:
- Message generation
- Support for publishers and subscriptions
- Loaned messages (zero-copy)
- Tunable QoS settings
- Clients and services

Lots of things are still missing however, see the [issue list](https://github.com/ros2-rust/ros2_rust/issues) for an overview. You are very welcome to [contribute](docs/CONTRIBUTING.md)!

Since the client library is still rapidly evolving, there are no stability guarantees for the moment.

Sounds great, how can I try this out?
-------------------------------------

Here are the steps for building the `ros2_rust` examples in a vanilla Ubuntu Focal installation. See the [in-depth guide for building `ros2_rust` packages](docs/building.md) for more details and options, including a Docker-based setup.

<!--- These steps should be kept in sync with docs/Building.md --->
```shell
# Install Rust, e.g. as described in https://rustup.rs/
# Install ROS 2 as described in https://docs.ros.org/en/foxy/Installation.html
# Assuming you installed the minimal version of ROS 2, you need these additional packages:
sudo apt install -y git libclang-dev python3-pip python3-vcstool # libclang-dev is required by bindgen
# Install these plugins for cargo and colcon:
cargo install --debug cargo-ament-build  # --debug is faster to install
pip install git+https://github.com/colcon/colcon-cargo.git
pip install git+https://github.com/colcon/colcon-ros-cargo.git

mkdir -p workspace/src && cd workspace
git clone https://github.com/ros2-rust/ros2_rust.git src/ros2_rust
vcs import src < src/ros2_rust/ros2_rust_foxy.repos
. /opt/ros/foxy/setup.sh
colcon build
```

Then, to run the minimal pub-sub example, do this:

```shell
# In a new terminal (or tmux window)
. ./install/setup.sh
ros2 run examples_rclrs_minimal_pub_sub minimal_publisher
# In a new terminal (or tmux window)
. ./install/setup.sh
ros2 run examples_rclrs_minimal_pub_sub minimal_subscriber
```
or

```shell
# In a new terminal (or tmux window)
. ./install/setup.sh
ros2 launch examples_rclrs_minimal_pub_sub minimal_pub_sub.launch.xml
```

Further documentation articles:
- [Tutorial on writing your first node with `rclrs`](docs/writing-your-first-rclrs-node.md)
- [Contributor's guide](docs/CONTRIBUTING.md)

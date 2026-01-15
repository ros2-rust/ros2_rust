ROS 2 for Rust
==============

[![Minimal Version Status](https://github.com/ros2-rust/ros2_rust/actions/workflows/rust-minimal.yml/badge.svg?branch=main)](https://github.com/ros2-rust/ros2_rust/actions/workflows/rust-minimal.yml)
[![Stable CI Status](https://github.com/ros2-rust/ros2_rust/actions/workflows/rust-stable.yml/badge.svg?branch=main)](https://github.com/ros2-rust/ros2_rust/actions/workflows/rust-stable.yml)
[![Win CI Status](https://github.com/ros2-rust/ros2_rust/actions/workflows/rust-win.yml/badge.svg?branch=main)](https://github.com/ros2-rust/ros2_rust/actions/workflows/rust-win.yml)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

Introduction
------------

This is a set of projects (the `rclrs` client library, code generator, examples and more) that
enables developers to write ROS 2 applications in Rust.

Features and limitations
------------------------

The current set of features include:
- Message generation with support for all ROS message types
- Publishers and subscriptions (including async variants)
- Loaned messages (zero-copy messaging)
- Dynamic message handling (runtime message introspection and manipulation)
- Tunable QoS settings
- Clients and services (including async variants)
- Actions (action servers and clients with async support)
- Timers (repeating, oneshot, and inert timers)
- Parameters (mandatory, optional, and read-only with parameter services)
- Logging with ROS-compliant logging utilities and rosout support
- Graph queries (topic/node discovery, endpoint information)
- Guard conditions and wait sets
- Clock and time APIs (including time sources)
- Worker pattern for managing shared state across callbacks
- Executor pattern for coordinating node execution

Some things are still missing however, see the [issue list](https://github.com/ros2-rust/ros2_rust/issues) for an overview. You are very welcome to [contribute](docs/CONTRIBUTING.md)!

Since the client library is still rapidly evolving, there are no stability guarantees for the moment.

Installation
------------

### Prerequisites

First, install Rust and the required system dependencies:

```shell
# Install Rust (see https://rustup.rs/)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Install required system packages
sudo apt install -y git libclang-dev python3-pip python3-vcstool

# Install colcon plugins for Rust
pip install git+https://github.com/colcon/colcon-cargo.git
pip install git+https://github.com/colcon/colcon-ros-cargo.git
```

### All ROS distributions

`rclrs` is released on [crates.io](https://crates.io/crates/rclrs), you can add it to your project directly:

```toml
[dependencies]
rclrs = "0.7"
```

Due to different ROS 2 distributions having different levels of integration with `rclrs`, if you'd like to
build and run the examples, please follow the according section.

### ROS 2 Lyrical Luth and Rolling

To run the examples, install the `test_msgs` and `example_interfaces` packages,
clone the examples repository to your workspace and build:

```shell
sudo apt install -y ros-rolling-example-interfaces ros-rolling-test-msgs
mkdir -p workspace/src && cd workspace
git clone https://github.com/ros2-rust/examples.git src/examples
. /opt/ros/rolling/setup.sh  # Or source your ROS 2 installation
colcon build
```

### ROS 2 Kilted Kaiju

For ROS 2 Kilted, clone the ROS 2 message packages to your workspace:

```shell
mkdir -p workspace/src && cd workspace
git clone -b kilted https://github.com/ros2/common_interfaces.git src/common_interfaces
git clone -b kilted https://github.com/ros2/example_interfaces.git src/example_interfaces
git clone -b kilted https://github.com/ros2/rcl_interfaces.git src/rcl_interfaces
git clone -b kilted https://github.com/ros2/test_msgs.git src/test_msgs
git clone -b kilted https://github.com/ros2/unique_identifier_msgs.git src/unique_identifier_msgs
```

Build the workspace:

```shell
. /opt/ros/kilted/setup.sh
colcon build
```

### ROS 2 Jazzy Jalisco

For ROS 2 Jazzy, you need to clone the code generator and message packages to your workspace:

```shell
mkdir -p workspace/src && cd workspace
git clone https://github.com/ros2-rust/rosidl_rust.git src/rosidl_rust
git clone -b jazzy https://github.com/ros2/common_interfaces.git src/common_interfaces
git clone -b jazzy https://github.com/ros2/example_interfaces.git src/example_interfaces
git clone -b jazzy https://github.com/ros2/rcl_interfaces.git src/rcl_interfaces
git clone -b jazzy https://github.com/ros2/test_msgs.git src/test_msgs
git clone -b jazzy https://github.com/ros2/unique_identifier_msgs.git src/unique_identifier_msgs
git clone https://github.com/ros2-rust/examples.git src/examples
```

Build the workspace:

```shell
. /opt/ros/jazzy/setup.sh
colcon build
```


### ROS 2 Humble Hawksbill

For ROS 2 Humble, you need to clone the code generator and message packages to your workspace:

```shell
mkdir -p workspace/src && cd workspace
git clone https://github.com/ros2-rust/rosidl_rust.git src/rosidl_rust
git clone -b humble https://github.com/ros2/common_interfaces.git src/common_interfaces
git clone -b humble https://github.com/ros2/example_interfaces.git src/example_interfaces
git clone -b humble https://github.com/ros2/rcl_interfaces.git src/rcl_interfaces
git clone -b humble https://github.com/ros2/test_msgs.git src/test_msgs
git clone -b humble https://github.com/ros2/unique_identifier_msgs.git src/unique_identifier_msgs
git clone https://github.com/ros2-rust/examples.git src/examples
```

Build the workspace:

```shell
. /opt/ros/humble/setup.sh
colcon build
```

### Running the examples

After building, source your workspace and run the examples:

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

For detailed building instructions and additional setup options, see the [in-depth guide](docs/building.md).

Further documentation articles:
- [Tutorial on writing your first node with `rclrs`](docs/writing-your-first-rclrs-node.md)
- [Contributor's guide](docs/CONTRIBUTING.md)

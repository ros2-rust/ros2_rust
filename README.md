ROS 2 for Rust
==============

[![Minimal Version Status](https://github.com/ros2-rust/ros2_rust/actions/workflows/rust-minimal.yml/badge.svg?branch=main)](https://github.com/ros2-rust/ros2_rust/actions/workflows/rust-minimal.yml)
[![Stable CI Status](https://github.com/ros2-rust/ros2_rust/actions/workflows/rust-stable.yml/badge.svg?branch=main)](https://github.com/ros2-rust/ros2_rust/actions/workflows/rust-stable.yml)
[![Win CI Status](https://github.com/ros2-rust/ros2_rust/actions/workflows/rust-win.yml/badge.svg?branch=main)](https://github.com/ros2-rust/ros2_rust/actions/workflows/rust-win.yml)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Introduction

This is a set of projects (the `rclrs` client library, code generator, examples, and more) that
enables developers to write ROS 2 applications in Rust.

## Features and Limitations

The current set of features includes:
- Message generation with support for all ROS message types
- Publishers and subscriptions (including async variants)
- Loaned messages (zero-copy messaging)
- Dynamic message handling (runtime message introspection and manipulation)
- Tunable QoS settings
- Clients and services (including async variants)
- Actions (action servers and clients with async support)
- Timers (repeating, one-shot, and inert timers)
- Parameters (mandatory, optional, and read-only with parameter services)
- Logging with ROS-compliant logging utilities and rosout support
- Graph queries (topic/node discovery, endpoint information)
- Guard conditions and wait sets
- Clock and time APIs (including time sources)
- Worker pattern for managing shared state across callbacks
- Executor pattern for coordinating node execution

Some things are still missing however, see the [issue list](https://github.com/ros2-rust/ros2_rust/issues) for an overview. You are very welcome to [contribute](docs/CONTRIBUTING.md)!

Since the client library is still rapidly evolving, there are no stability guarantees for the moment.

## Installation

### Prerequisites

First, install Rust and the required system dependencies:

```shell
# Install Rust (see https://rustup.rs/)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Install required system packages
sudo apt install -y git libclang-dev python3-pip python3-vcstool

# Install colcon plugins for Rust
pip install --break-system-packages colcon-cargo colcon-ros-cargo
```


### All ROS Distributions

`rclrs` is released on [crates.io](https://crates.io/crates/rclrs) and you can add it to your project directly:

```toml
[dependencies]
rclrs = "0.8"
```

To build and run the examples, install the `example_interfaces` and `test_msgs` ROS packages, then clone the examples repository into a workspace:

```shell
sudo apt install -y ros-$ROS_DISTRO-example-interfaces ros-$ROS_DISTRO-test-msgs
mkdir -p workspace/src && cd workspace
git clone https://github.com/ros2-rust/examples.git src/examples
```

Build the workspace:

```shell
. /opt/ros/$ROS_DISTRO/setup.sh  # Or source your ROS 2 installation
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

Or:

```shell
# In a new terminal (or tmux window)
. ./install/setup.sh
ros2 launch examples_rclrs_minimal_pub_sub minimal_pub_sub.launch.xml
```

For detailed building instructions and additional setup options, see the [in-depth guide](docs/building.md).

## AI Policy
Generative tools are allowed in producing contributions to its projects, with some qualifications:

- Any contribution may consist, in whole or in part, of the output of one or more generative tools.
- Any use of generative tools in a contribution must be disclosed at the time of making the contribution.
- The disclosure must be recorded in a way that ensures it has the same or greater lifetime as the contribution itself.

For source code contributions, you should add a disclosure statement in the commit message for all commits where some portion of the source code was generated.

`Assisted-by: AGENT_NAME:MODEL_VERSION [TOOL1] [TOOL2]`

Provide a similar disclosure statement in the PR description.

See the projects [AI Policy](docs/AI_POLICY.md) for more details.

## Further Documentation

- [Tutorial on writing your first node with `rclrs`](docs/writing-your-first-rclrs-node.md)
- [Contributor's guide](docs/CONTRIBUTING.md)

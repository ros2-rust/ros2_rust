ROS2 for Rust
=============

Build status
------------

| Target | Status |
|----------|--------|
| **Ubuntu 20.04** | [![Build Status](https://github.com/ros2-rust/ros2_rust/actions/workflows/rust.yml/badge.svg?branch=master)](https://github.com/ros2-rust/ros2_rust/actions/workflows/rust.yml?branch=master) |

Introduction
------------

This is a set of projects (bindings, code generator, examples and more) that enables developers to write ROS2
applications in Rust.

Features
--------

The current set of features include:
- Generation of all builtin ROS types
- Support for publishers and subscriptions
- Tunable QoS settings

What's missing?
---------------

Lots of things!
- Component nodes
- Clients and services
- Tests
- Documentation

### Limitations

- messages are deep-copied and this can be terribly inefficient for big messages like images; the current solution leverages C typesupport implementations and might benefit from a direct serialization/deserialization
- `rclrs` interface is very limited for now and might not be so much idiomatic yet, any help and suggestion on the interface would be greatly appreciated
- due to the current ROS2 support of non-default clients, packages containing definitions of messages used in Rust crates must be present in the current workspace; otherwise message crates generation won't be triggered

Sounds great, how can I try this out?
-------------------------------------

The following steps were last tested on Ubuntu 20.04. Here, the Foxy distribution of ROS 2 is used, but newer distributions can be used by simply replacing 'foxy' with the distribution name.

```
# First, make sure to have ROS 2 and vcstool installed (alternatively, install vcstool with pip):
# sudo apt install ros-foxy-desktop ros-foxy-test-interface-files python3-vcstool libclang-dev clang
# Install the colcon-cargo and colcon-ros-cargo plugins
pip install git+https://github.com/colcon/colcon-cargo.git
pip install git+https://github.com/colcon/colcon-ros-cargo.git
# Install the cargo-ament-build plugin
cargo install cargo-ament-build

# In your workspace directory (ideally an empty one), run 
mkdir src
git clone https://github.com/ros2-rust/ros2_rust.git src/ros2_rust
vcs import src < src/ros2_rust/ros2_rust_foxy.repos
. /opt/ros/foxy/setup.sh
colcon build --packages-up-to rclrs_examples
```

It's normal to see a `Some selected packages are already built in one or more underlay workspace` warning. This is because the standard message definitions that are part of ROS 2 need to be regenerated in order to create Rust bindings.

Now you can just run a bunch of examples.

### Publisher and subscriber

Publisher:

```
# Do this in a new terminal
. ./install/setup.sh
ros2 run rclrs_examples rclrs_publisher
```

Subscriber:

```
# Do this in a new terminal
. ./install/setup.sh
ros2 run rclrs_examples rclrs_subscriber
```

Enjoy!

If something goes very wrong and you want to start fresh, make sure to delete all `install*`, `build*` and `.cargo` directories. Also, make sure your terminal does not have any install sourced (check with `echo $AMENT_PREFIX_PATH`, which should be empty).

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
- An ament build type for Cargo. The current examples use CMake to install and build the binaries... and it's really ugly.
- Component nodes
- Clients and services
- Tests
- Documentation

### Limitations

- messages are deep-copied and this can be terribly inefficient for big messages like images; the current solution leverages C typesupport implementations and might benefits from a direct serialization/deserialization
- the current solution for crates export with CMake is not very robust
- `rclrs` interface is very limited for now and might not be so much idiomatic yet, any help and suggestion on the interface would be greatly appreciated
- due to the current ROS2 support of non-default clients, packages containing definitions of messages used in Rust crates must be present in the current workspace; otherwise message crates generation won't be triggered

Sounds great, how can I try this out?
-------------------------------------

You can build and run the example using the included Dockerfile: 

```
git clone https://github.com/ros2-rust/ros2_rust.git
docker build --tag ros2:rust .
docker run -it --rm ros2:rust
ros2 run rclrs_examples rclrs_publisher &
ros2 run rclrs_examples rclrs_subscriber

```

Or do so manually as summarized in the steps below:

> The following steps were last tested on Ubuntu 20.04.

```
# first, install vcstool from PyPI or apt:
# sudo apt install ros-foxy-desktop ros-foxy-test-interface-files python3-vcstool libclang-dev clang
# pip install vcstool

mkdir -p ~/ros2_rust_ws/src/ros2-rust
cd ~/ros2_rust_ws/src/ros2-rust
git clone https://github.com/ros2-rust/ros2_rust.git
cd ../../
vcs import src < src/ros2-rust/ros2_rust/ros2_rust.repos
source /opt/ros/foxy/setup.sh
colcon build
```

Now you can just run a bunch of examples.

### Publisher and subscriber

Publisher:

```
. ./install/setup.sh

ros2 run rclrs_examples rclrs_publisher
```

Subscriber:

```
. ./install/setup.sh

ros2 run rclrs_examples rclrs_subscriber
```

Enjoy!

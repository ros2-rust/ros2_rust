ROS2 for Rust
=============

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

The following steps show how to build the examples:

```
# first, install vcstool from PyPI or apt:
# sudo apt install python3-vcstool
# pip install vcstool

mkdir -p ~/ros2_rust_ws/src
cd ~/ros2_rust_ws
wget https://raw.githubusercontent.com/ros2-rust/ros2_rust/master/ros2_rust.repos
vcs import src < ros2_rust.repos
source /opt/ros/crystal/setup.sh
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

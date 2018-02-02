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
- Nested types
- An ament build type for Cargo. The current examples use CMake to install and build the binaries... and it's really ugly.
- Component nodes
- Clients and services
- Tests
- Documentation

Sounds great, how can I try this out?
-------------------------------------

The following steps show how to build the examples:

```
mkdir -p ~/ros2_rust_ws/src
cd ~/ros2_rust_ws
wget https://raw.githubusercontent.com/esteve/ros2_rust/master/ros2_rust.repos
vcs import ~/ros2_rust_ws/src < ros2_rust.repos
cd ~/ros2_rust_ws/src/ros2/rosidl_typesupport
patch -p1 < ../../ros2_rust/ros2_rust/rosidl_typesupport_ros2_rust.patch
cd ~/ros2_rust_ws
src/ament/ament_tools/scripts/ament.py build --isolated
```

Now you can just run a bunch of examples.

### Publisher and subscriber

Publisher:

```
. ~/ros2_rust_ws/install_isolated/local_setup.sh

ros2 run rclrs_examples rclrs_publisher
```

Subscriber:

```
. ~/ros2_rust_ws/install_isolated/local_setup.sh

ros2 run rclrs_examples rclrs_subscriber
```

Enjoy!

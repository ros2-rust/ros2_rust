# ROS2 for Rust

## Overview

This is a set of projects (bindings, code generator, examples and more) that enables developers to write ROS2
applications in Rust.

This is a fork of [github.com/esteve/ros2_rust](https://github.com/esteve/ros2_rust).

## Status

This project is still in early development stage. It's just another step toward a first class Rust support in ROS2.

### Changes from original repository

- no need to patch ROS2 installation
- `rosidl_generator_rs` is extended to support nested messages and arrays in messages, so all message types should be supported now
- the new crate `rcl_sys` use [bindgen](https://github.com/rust-lang/rust-bindgen) to automatically generate RCL binding based on ROS2 installation
- `rclrs` and `rclrs_examples` are totally rewritten to use `rcl_sys` and try to wrap RCL into an idiomatic Rust interface
- ROS2 Crystal release is supported

### Limitations

- messages are deep-copied and this can be terribly inefficient for big messages like images; the current solution leverages C typesupport implementations and might benefits from a direct serialization/deserialization
- the current solution for crates export with CMake is not very robust
- `rclrs` interface is very limited for now and might not be so much idiomatic yet, any help and suggestion on the interface would be greatly appreciated
- due to the current ROS2 support of non-default clients, packages containing definitions of messages used in Rust crates must be present in the current workspace; otherwise message crates generation won't be triggered
- due to a not yet understood issue, `find_package` inclusion order matters in `CMakeLists.txt` of ROS packages containing message definitions:
    `rosidl_default_generators` must be included before any other ROS packages containing message definitions, as in the following sample from `std_msgs`
    ```cmake
    find_package(rosidl_default_generators REQUIRED)
    find_package(builtin_interfaces REQUIRED)
    ```

## Quickstart

### Build

```bash
source /opt/ros/crystal/setup.bash
colcon build --symlink-install --merge-install --cmake-args -G Ninja
```

### Publisher

```bash
source ./install/setup.bash
ros2 run rclrs_examples rclrs_publisher
```

### Subscriber

```bash
source ./install/setup.bash
ros2 run rclrs_examples rclrs_subscriber
```
# rclrs examples

Each directory is a ROS package that shows one part of `rclrs`.
`rclrs_example_msgs` defines the messages that `message_demo` uses.

The examples are members of the cargo workspace at the repository root, so they always build against the `rclrs` and `ros-env` sources in this repository.
On `main` they can use features that are not released yet. For the examples that match a released `rclrs`, check out its `rclrs-vX.Y.Z` tag. Examples for releases up to 0.8.0 are in the [examples repository](https://github.com/ros2-rust/examples).

## Building and running

Build them with colcon from your workspace directory. colcon only sees the examples and `rclrs_example_msgs`, the library crates are built by cargo as dependencies.

```shell
. /opt/ros/$ROS_DISTRO/setup.sh
colcon build
. install/setup.sh
ros2 run examples_rclrs_minimal_pub_sub minimal_publisher
```

Plain cargo works as well once `rclrs_example_msgs` is in a sourced workspace:

```shell
cd src/ros2_rust
cargo run -p examples_rclrs_message_demo
```

The examples are not part of the default workspace members, so a plain `cargo build` at the repository root skips them.
If the bindings that your distribution ships are too old for `rclrs` on `main`, build the interface packages from source as described in [Importing repositories](../docs/building.md#importing-repositories).

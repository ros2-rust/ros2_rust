# ros-env
Top-level Rust crate to include generated Rust code found in a sourced ROS 2 workspace.

## Usage
The [shape/msg/Plane](https://github.com/ros2/common_interfaces/blob/rolling/shape_msgs/msg/Plane.msg) message can be included with:
```rust
// Assuming the rust crate for `shape_msgs` is in the `AMENT_PREFIX_PATH`
use ros_env::shape_msgs::msg::Plane;
```

## Details
Any Rust crate found in the `AMENT_PREFIX_PATH` environment variable, that has opted in, will be `include!()`d.

To opt in, the crate must have the following metadata present in the Cargo.toml
```toml
[package.metadata.ros-env]
include = true
```

By default, crates generated from `rosidl_generator_rs` opt in.

## Limitations
- The [include!()](https://doc.rust-lang.org/std/macro.include.html) macro is literal text inclusion. As such, depending 
  on the number of generated crates found in `AMENT_PREFIX_PATH`, the build times for this crate can be long.
- The dependencies of the included crates are not included. You cannot dynamically alter cargo dependencies through 
  anything other than features, and features need to be explicitly declared and enabled. As such, this crate must have 
  all expected dependencies itself (hence why this crate has a `serde` dependency for example).

## AI Policy
Generative tools are allowed in producing contributions to its projects, with some qualifications:

- Any contribution may consist, in whole or in part, of the output of one or more generative tools.
- Any use of generative tools in a contribution must be disclosed at the time of making the contribution.
- The disclosure must be recorded in a way that ensures it has the same or greater lifetime as the contribution itself.

For source code contributions, you should add a disclosure statement in the commit message for all commits where some portion of the source code was generated.

`Assisted-by: AGENT_NAME:MODEL_VERSION [TOOL1] [TOOL2]`

Provide a similar disclosure statement in the PR description.

See the projects [AI Policy](https://github.com/ros2-rust/ros2_rust/blob/main/docs/AI_POLICY.md) for more details.
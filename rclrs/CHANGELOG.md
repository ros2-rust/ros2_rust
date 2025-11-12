# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.6.0](https://github.com/ros2-rust/ros2_rust/compare/v0.5.1...v0.6.0) - 2025-10-27

### Added

- [**breaking**] async actions ([#503](https://github.com/ros2-rust/ros2_rust/pull/503))
- timers API ([#480](https://github.com/ros2-rust/ros2_rust/pull/480))

### Other

- re-add generate_bindings.py script
- Minimal changes for new Action trait ([#539](https://github.com/ros2-rust/ros2_rust/pull/539))
- Re-export traits from rosidl_runtime_rs ([#537](https://github.com/ros2-rust/ros2_rust/pull/537))
- Fix use of serde ([#538](https://github.com/ros2-rust/ros2_rust/pull/538))
- Put `use rustflags` next to its usage to prevent a build warning. ([#526](https://github.com/ros2-rust/ros2_rust/pull/526))

## [0.5.1](https://github.com/ros2-rust/ros2_rust/compare/v0.5.0...v0.5.1) - 2025-08-23

### Other

- Fix executor timeout ([#519](https://github.com/ros2-rust/ros2_rust/pull/519))

## [0.5.0](https://github.com/ros2-rust/ros2_rust/compare/v0.4.1...v0.5.0) - 2025-08-15

### Added

- vendorize messages so that cargo update works ([#509](https://github.com/ros2-rust/ros2_rust/pull/509))

### Other

- Moved examples to its own repo ([#504](https://github.com/ros2-rust/ros2_rust/pull/504))
- Wake up wait set when adding a new waitable ([#505](https://github.com/ros2-rust/ros2_rust/pull/505))
- rename generate_docs feature to use_ros_shim ([#501](https://github.com/ros2-rust/ros2_rust/pull/501))
- Async Workers ([#446](https://github.com/ros2-rust/ros2_rust/pull/446))
- Shared state pattern (spin-off of #427) ([#430](https://github.com/ros2-rust/ros2_rust/pull/430))
- Options pattern (spin-off of #427) ([#429](https://github.com/ros2-rust/ros2_rust/pull/429))
- Fix RAII for subscription, client, and service ([#463](https://github.com/ros2-rust/ros2_rust/pull/463))
- Execution structure (spin-off of #427) ([#428](https://github.com/ros2-rust/ros2_rust/pull/428))
- Remove Iron support ([#443](https://github.com/ros2-rust/ros2_rust/pull/443))
- Add Publisher::get_subscription_count ([#457](https://github.com/ros2-rust/ros2_rust/pull/457))
- Bumb bindgen dependency to 0.70 ([#452](https://github.com/ros2-rust/ros2_rust/pull/452))
- Update for clippy 1.83 ([#441](https://github.com/ros2-rust/ros2_rust/pull/441))
- Add Publisher::can_loan_msgs ([#434](https://github.com/ros2-rust/ros2_rust/pull/434))
- Add rosout logging to rclrs ([#422](https://github.com/ros2-rust/ros2_rust/pull/422))
- Update vendored interface packages ([#423](https://github.com/ros2-rust/ros2_rust/pull/423))
- Use latest stable Rust CI + Fix Test Errors ([#420](https://github.com/ros2-rust/ros2_rust/pull/420))
- Add std::error::Error impls to Error enums of `parameter` module ([#413](https://github.com/ros2-rust/ros2_rust/pull/413))
- Wrap slice::from_raw_parts to be compatible with rcl ([#419](https://github.com/ros2-rust/ros2_rust/pull/419))
- Compile on targets where c_char ≠ i8 ([#403](https://github.com/ros2-rust/ros2_rust/pull/403))
- Add parameter services ([#342](https://github.com/ros2-rust/ros2_rust/pull/342))
- Fixup of #388 ([#389](https://github.com/ros2-rust/ros2_rust/pull/389))
- Fix link formatting
- Get rid of doc links to private structs
- Update documentation on ENTITY_LIFECYCLE_MUTEX
- Improve the documentation for the domain ID situation of test_graph_empty
- Rename to avoid confusion with Handle pattern
- Update documentation and safety info on rcl entity lifecycles
- Remove the need for lazy_static
- Satisfy clippy
- Use usize instead of u8 for domain id
- Ensure that test_graph_empty works even if the system has ROS_DOMAIN_ID set to 99
- Run clippy
- Run rustfmt
- Apply lifecycle lock to all middleware entities
- Ensure that mutex guards are not being dropped prematurely
- Introduce InitOptions to allow manually setting domain ID
- Keep context alive for guard conditions
- Manage all rcl bindings with Handle structs
- Reworking the lifecycle management of rcl bindings
- Import the builtin_interfaces directly from the internal vendor module.
- Added `test_msgs` as a test dependency
- Move the tests in the `rclrs_tests` crate into `rclrs`.
- Move rcl structs to end of Node declaration
- Allow ros2_rust to be built within a distro workspace
- Add default implementation and builder pattern for QoS ([#361](https://github.com/ros2-rust/ros2_rust/pull/361))
- Adding a simple helper function for converting `rclrs::Time` to a ros message ([#359](https://github.com/ros2-rust/ros2_rust/pull/359))
- Remove leading underscore from private fields ([#354](https://github.com/ros2-rust/ros2_rust/pull/354))

## [0.4.1](https://github.com/ros2-rust/ros2_rust/compare/v0.4.0...v0.4.1) - 2023-11-28
- Added minor changes to enable documentation generation on docs.rs for the `ros2-rust` projects.

## [0.4.0](https://github.com/ros2-rust/ros2_rust/compare/v0.3.1...v0.4.0) - 2023-11-07
- Service clients now support service_is_ready to check if a service server is present ahead of calling ([#399](https://github.com/ros2-rust/ros2_rust/pull/339))
- Added preliminary support for parameters
- Added support for Iron Irwini
- Added Serde big array support
- Added basic functionality for loading introspection type support libraries
- Added extended string types
- Added time source and clock API to nodes
- Removed support for Galactic
- Removed support for Foxy

## [0.3.1](https://github.com/ros2-rust/ros2_rust/compare/v0.3.0...v0.3.1) - 2022-10-17
- Fixed segfault when re-using `WaitSet`
- Fixed `Node::get_{publishers,subscriptions}_info_by_topic()`

## [0.3.0](https://github.com/ros2-rust/ros2_rust/compare/v0.2.0...v0.3.0) - 2022-10-03
- Loaned messages (zero-copy) ([#212](https://github.com/ros2-rust/ros2_rust/pull/212))
- Graph queries ([#234](https://github.com/ros2-rust/ros2_rust/pull/234))
- Guard conditions ([#249](https://github.com/ros2-rust/ros2_rust/pull/249))

## [0.2.0] (2022-07-21)
- First release
- `colcon-cargo` and `colcon-ros-cargo` can now build any pure Cargo and ament-aware Cargo projects
- `rclrs` and `rclrs_examples` are now `ament_cargo` projects, no more CMake involved
- `rosidl_generator_rs` has been updated to support all ROS message types
- `rclrs` now supports clients and services
- Better API documentation
- Foxy, Galactic, Humble and Rolling are now supported ROS distros
- Preliminary support for Windows
- Build based on `colcon-ros-cargo`
- Message generation packages `rosidl_generator_rs` and `rosidl_runtime_rs`
- Publisher, Subscription, Client and Service
- Tunable QoS settings
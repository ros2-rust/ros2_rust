# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.7.0](https://github.com/ros2-rust/rosidl_runtime_rs/compare/v0.6.1...v0.7.0) - 2026-09-05

### Fixed

- [**breaking**] match rosidl_buffer primitive-sequence ABI on Lyrical+ ([#22](https://github.com/ros2-rust/rosidl_runtime_rs/pull/22))
- don't hard code a *nix path separator ([#27](https://github.com/ros2-rust/rosidl_runtime_rs/pull/27))

## [0.6.1](https://github.com/ros2-rust/rosidl_runtime_rs/compare/v0.6.0...v0.6.1) - 2026-06-08

### Other

- *(sequence)* add From<Sequence<T>> for Vec<T> with Copy bound ([#20](https://github.com/ros2-rust/rosidl_runtime_rs/pull/20))

## [0.6.0](https://github.com/ros2-rust/rosidl_runtime_rs/compare/v0.5.0...v0.6.0) - 2026-01-14

### Other

- [**breaking**] require rustc 1.85 ([#18](https://github.com/ros2-rust/rosidl_runtime_rs/pull/18))

## [0.5.0](https://github.com/ros2-rust/rosidl_runtime_rs/compare/v0.4.2...v0.5.0) - 2025-10-12

### Other

- Improve Action trait ([#15](https://github.com/ros2-rust/rosidl_runtime_rs/pull/15))

## [0.4.2](https://github.com/ros2-rust/rosidl_runtime_rs/compare/v0.4.1...v0.4.2) - 2025-07-11

### Fixed

- add Cargo.lock to version control ([#3](https://github.com/ros2-rust/rosidl_runtime_rs/pull/3))

### Other

- add release-plz CI action ([#5](https://github.com/ros2-rust/rosidl_runtime_rs/pull/5))
- added changelog
- Update for clippy 1.83 ([#441](https://github.com/ros2-rust/rosidl_runtime_rs/pull/441))
- Action message support ([#417](https://github.com/ros2-rust/rosidl_runtime_rs/pull/417))
- Add in missing nullptr check when calling `std::slice::from_raw_parts` ([#416](https://github.com/ros2-rust/rosidl_runtime_rs/pull/416))
- Fix panic on sequence_init() when size is 0 ([#407](https://github.com/ros2-rust/rosidl_runtime_rs/pull/407))
- Add parameter services ([#342](https://github.com/ros2-rust/rosidl_runtime_rs/pull/342))
- Use nightly for style check ([#396](https://github.com/ros2-rust/rosidl_runtime_rs/pull/396))
- Fix Unicode handling in String and WString ([#362](https://github.com/ros2-rust/rosidl_runtime_rs/pull/362))

# Contributing
Contributions to `ros2_rust` are very welcome!

Work is coordinated in the [Matrix chat](https://matrix.to/#/+rosorg-rust:matrix.org). It's a good idea to check in with the other contributors there before starting development on a new feature.

There are also occasional ROS 2 Rust WG meetings that are announced on the [ROS discourse forum](https://discourse.ros.org/).

## Coding guidelines
Aim for well documented and tested code.

Use declarative sentences, not imperative sentences, for documenting functions. For instance, `Creates a by-value iterator.`, not `Create a by-value iterator`. 

Use `cargo clippy` to help with writing idiomatic code.

### Unsafe code
Annotate every `unsafe` block with a `// SAFETY:` comment explaining why its safety preconditions are met.

### Formatting
Use `cargo fmt` to format code.

Put a struct's definition first, then its trait impls in alphabetical order, then its inherent impls. When there are several structs in one file, one possibility is to put all struct definitions at the top of the file, like this:
- `struct A {…}`
- `struct B {…}`
- `impl Bar for A {…}`
- `impl Foo for A {…}`
- `impl A {…}`
- `impl Bar for B {…}`
- `impl Foo for B {…}`
- `impl B {…}`

Try to not exceed a line length of 100 characters.

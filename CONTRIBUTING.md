# Contributing
Contributions to `ros2_rust` are very welcome!

Work is coordinated in the [Matrix chat](https://matrix.to/#/+rosorg-rust:matrix.org). It's a good idea to check in with the other contributors there before starting development on a new feature.

There are also occasional ROS 2 Rust WG meetings that are announced on the [ROS discourse forum](https://discourse.ros.org/).


## Coding guidelines
This section is not comprehensive, and conventions from the broader Rust community should be followed.

Use `cargo clippy` to check that code is idiomatic.

For documenting functions, use declarative sentences, not imperative sentences. For instance, `Creates a by-value iterator.` should be used instead of `Create a by-value iterator.`.

### Unsafe code
Keep `unsafe` blocks as small as possible.
Annotate every `unsafe` block with a `// SAFETY:` comment explaining why its safety preconditions are met.

### Formatting
Use `cargo fmt` to format code.

Put a `struct`'s definition first, then its trait `impl`s in alphabetical order, then its inherent `impl`s. When there are several `struct`s in one file, one possibility is to put all `struct` definitions at the top of the file, like this:
- `struct A {…}`
- `struct B {…}`
- `impl Bar for A {…}`
- `impl Foo for A {…}`
- `impl A {…}`
- `impl Bar for B {…}`
- `impl Foo for B {…}`
- `impl B {…}`

Try to not exceed a line length of 100 characters for comments.


## License
Any contribution that you make to this repository will
be under the Apache 2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0.html):

~~~
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
~~~
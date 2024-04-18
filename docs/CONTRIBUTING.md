# Contributing to `ros2_rust`
Contributions to `ros2_rust` are very welcome!

Work is coordinated in via GitHub issues, and there also exists a [Matrix chat](https://matrix.to/#/+rosorg-rust:matrix.org) for general discussion. It's a good idea to check in with the other contributors and seek early-stage feedback when developing a major feature.

There are also occasional ROS 2 Rust WG meetings that are announced on the [ROS discourse forum](https://discourse.ros.org/).


## Coding guidelines
This section is not comprehensive, and conventions from the broader Rust community should be followed.

If you're looking for resources to learn idiomatic Rust, a good starting point is https://github.com/mre/idiomatic-rust.

Use `cargo clippy` to check that code is idiomatic.

### Documentation
For documenting functions, use declarative sentences, not imperative sentences. For instance, `Creates a by-value iterator.` should be used instead of `Create a by-value iterator.`. Use a period at the end of every phrase, even the "brief" description.

Document any panics that might happen in a function, and where it makes sense, also document the errors that may be returned by a function.

When linking to documents from the ROS 2 ecosystem, always use the a link that redirects to the latest version, or a link to the Rolling version.

### Unsafe code
Keep `unsafe` blocks as small as possible.

Annotate every `unsafe` block with a `// SAFETY:` comment explaining why it is safe. For function calls, that should cover the function's preconditions, if any, and any other interesting considerations. For instance, if you'd call [`rcl_context_get_init_options()`](https://github.com/ros2/rcl/blob/4b125b1af0e2e2c8c7dd0c8e18b5a8d36709058c/rcl/include/rcl/context.h#L217), the comment should explain whether the pointer that is returned by the function needs to be freed by the caller later, which hinges on whether it's a deep copy of the init options stored inside the context. If it's not a deep copy, it should be explained how it is guaranteed that the init options pointer does not outlive the context (e.g. because it is immediately converted to an owned type).

In many cases, the only precondition is that the function arguments are valid pointers, which is trivially true if they are created from references. In such cases, a very brief safety comment such as "pointers are trivially valid" is sufficient.

Inside `unsafe` functions, you can use `/* unsafe */` comments as a substitute for the `unsafe` keyword.

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

### Error messages
Error messages should
- be capitalized
- not have punctuation at the end
- not include an "Error:" prefix or similar

### Tests
Prefer using `quickcheck` tests and doctests where they make sense.

Avoid writing tests that simply restate the definition of a function. E.g. for `fn square(x: i32) { x*x }` do not write a test that asserts `square(3) == 3*3`.

### Return type to use when wrapping `rcl` functions
Functions from `rcl` can typically return error codes. This does not necessarily mean that the `rclrs` function using it must return `Result`. If the error conditions can be determined, from reading `rcl` comments and source code, to never occur, the return code doesn't need to be propagated but only `debug_assert!`ed to be ok. This assert guards against `rcl` introducing a new error condition in the future.

An example would be a function that only returns an error when the pointer we pass in is null or points to an uninitialized object. If we obtain the pointer from a reference, it can't be null, and if the object that is referenced is guaranteed to be initialized, there is no need to return a `Result`.

### References vs raw pointers

Do not cast references to raw pointers where it's not needed. Since references coerce to raw pointers, they can be directly used as arguments to C functions in most cases.

### Implementing the `Drop` trait

Generally, a `rcl` type should have a `Drop` impl that calls its `fini()` function. However, there are some cases where this is not possible, e.g. when the type is part of another `rcl` type that owns and finalizes it, or when the `fini()` function takes extra arguments. In these cases, it needs to be decided individually if and where to call `fini()`, and care must be taken to not return early from a function with `?` before the relevant `fini()` functions have been called.


## Consistency with major ROS 2 client libraries
Consistency with the two major ROS 2 client libraries, `rclcpp` and `rclpy`, is a goal of this project – and in particular `rclcpp`.

That means that
- Roughly the same concepts should exist in all client libraries, e.g. services, parameters, etc.
- Types, functions, argument names etc. should have the same or similar names as their corresponding items in `rclcpp`/`rclpy` where possible
- Code should be structured into similar packages by default, e.g. `rosidl_generator_<lang>`, `rcl<lang>` etc.
- Features should be built on top of `rcl` by default, instead of re-implementing them from scratch

This does _not_ mean that
- No variability between languages is allowed. What is easy to do, and what is considered idiomatic, is quite different between languages, and thus ROS 2 allows for considerable variability between client languages. Excessively complex designs in C++ or Python may be caused by limitations of such languages and it's best to strive for clean, simple functionality in Rust, possibly leveraging zero-cost abstractions that are unique to the language.
- Features should always be ported 1:1. Do not rely on the assumption that the best implementation for `rclrs` is whatever `rclcpp` does, but understand the motivation and status of the implementation in the other client libraries. For instance, sometimes a design is implemented incompletely by a client library, the feature is later found to have problems (e.g. the deadlocking of sync client calls in Python), or the client library is constrained by backwards compatibility.
- Consistency within `rclrs` does not matter. It does.
- No deviations from the above guidelines are possible.

In summary, understand the context of the features that you port, in order to avoid [_cargo-culting_](https://en.wikipedia.org/wiki/Cargo_cult_programming).


## Squashing and amending commits
As soon as a PR is in review, it is generally preferable to not squash and amend commits anymore without the agreement of the reviewer.
The reason is that changes – especially large ones – are easier to follow when they are added in new commits, and that those commits can be referenced in review discussions.
When the PR is merged, all commits in the PR are squashed anyway.


## Reviewing code
When reviewing pull requests, try to understand and give feedback on the high-level design first, before commenting on coding style, safety comments and such.

Use the "changes requested" status sparingly, and instead consider simply leaving comments. This enables other reviewers to approve the pull requests when they see that all requested changes have been made. Conversely, if you are a reviewer and see that someone else has left comments requesting changes, it's expected that you ensure that all of them have been addressed before you approve.


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

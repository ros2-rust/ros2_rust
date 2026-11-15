^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosidl_runtime_rs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Update for clippy 1.83 (`#441 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/441>`_)
  * Update for clippy 1.83
  * Update clippy for cfg(test) as well
  * Exclude dependencies from clippy test
  * Fix clippy for rosidl_runtime_rs
  ---------
* Action message support (`#417 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/417>`_)
  * Added action template
  * Added action generation
  * Added basic create_action_client function
  * dded action generation
  * checkin
  * Fix missing exported pre_field_serde field
  * Removed extra code
  * Sketch out action server construction and destruction
  This follows generally the same pattern as the service server. It
  required adding a typesupport function to the Action trait and pulling
  in some more rcl_action bindings.
  * Fix action typesupport function
  * Add ActionImpl trait with internal messages and services
  This is incomplete, since the service types aren't yet being generated.
  * Split srv.rs.em into idiomatic and rmw template files
  This results in the exact same file being produced for services,
  except for some whitespace changes. However, it enables actions to
  invoke the respective service template for its generation, similar to
  how the it works for services and their underlying messages.
  * Generate underlying service definitions for actions
  Not tested
  * Add runtime trait to get the UUID from a goal request
  C++ uses duck typing for this, knowing that for any `Action`, the type
  `Action::Impl::SendGoalService::Request` will always have a `goal_id`
  field of type `unique_identifier_msgs::msg::UUID` without having to
  prove this to the compiler. Rust's generics are more strict, requiring
  that this be proven using type bounds.
  The `Request` type is also action-specific as it contains a `goal` field
  containing the `Goal` message type of the action. We therefore cannot
  enforce that all `Request`s are a specific type in `rclrs`.
  This seems most easily represented using associated type bounds on the
  `SendGoalService` associated type within `ActionImpl`. To avoid
  introducing to `rosidl_runtime_rs` a circular dependency on message
  packages like `unique_identifier_msgs`, the `ExtractUuid` trait only
  operates on a byte array rather than a more nicely typed `UUID` message
  type.
  I'll likely revisit this as we introduce more similar bounds on the
  generated types.
  * Integrate RMW message methods into ActionImpl
  Rather than having a bunch of standalone traits implementing various
  message functions like `ExtractUuid` and `SetAccepted`, with the
  trait bounds on each associated type in `ActionImpl`, we'll instead add
  these functions directly to the `ActionImpl` trait. This is simpler on
  both the rosidl_runtime_rs and the rclrs side.
  * Add rosidl_runtime_rs::ActionImpl::create_feedback_message()
  Adds a trait method to create a feedback message given the goal ID and
  the user-facing feedback message type. Depending on how many times we do
  this, it may end up valuable to define a GoalUuid type in
  rosidl_runtime_rs itself. We wouldn't be able to utilize the
  `RCL_ACTION_UUID_SIZE` constant imported from `rcl_action`, but this is
  pretty much guaranteed to be 16 forever.
  Defining this method signature also required inverting the super-trait
  relationship between Action and ActionImpl. Now ActionImpl is the
  sub-trait as this gives it access to all of Action's associated types.
  Action doesn't need to care about anything from ActionImpl (hopefully).
  * Add GetResultService methods to ActionImpl
  * Implement ActionImpl trait methods in generator
  These still don't build without errors, but it's close.
  * Replace set_result_response_status with create_result_response
  rclrs needs to be able to generically construct result responses,
  including the user-defined result field.
  * Implement client-side trait methods for action messages
  This adds methods to ActionImpl for creating and accessing
  action-specific message types. These are needed by the rclrs
  ActionClient to generically read and write RMW messages.
  Due to issues with qualified paths in certain places
  (https://github.com/rust-lang/rust/issues/86935), the generator now
  refers directly to its service and message types rather than going
  through associated types of the various traits. This also makes the
  generated code a little easier to read, with the trait method signatures
  from rosidl_runtime_rs still enforcing type-safety.
  * Format the rosidl_runtime_rs::ActionImpl trait
  * Wrap longs lines in rosidl_generator_rs action.rs
  This at least makes the template easier to read, but also helps with the
  generated code. In general, the generated code could actually fit on one
  line for the function signatures, but it's not a big deal to split it
  across multiple.
  * Use idiomatic message types in Action trait
  This is user-facing and so should use the friendly message types.
  * Cleanup ActionImpl using type aliases
  * Formatting
  * Switch from std::os::raw::c_void to std::ffi::c_void
  While these are aliases of each other, we might as well use the more
  appropriate std::ffi version, as requested by reviewers.
  * Clean up rosidl_generator_rs's cmake files
  Some of the variables are present but no longer used. Others were not
  updated with the action changes.
  * Add a short doc page on the message generation pipeline
  This should help newcomers orient themselves around the rosidl\_*_rs
  packages.
  ---------
  Co-authored-by: Esteve Fernandez <esteve@apache.org>
  Co-authored-by: Michael X. Grey <grey@openrobotics.org>
* Add in missing nullptr check when calling `std::slice::from_raw_parts` (`#416 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/416>`_)
  * Add in missing nullptr check when calling `std::slice::from_raw_parts`
  * Added missing testcase
* Fix panic on sequence_init() when size is 0 (`#407 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/407>`_)
* Add parameter services (`#342 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/342>`_)
  * WIP Adding describe paramater service
  * Implement parameter setting services
  * Restructure and cleanup
  * Implement list_parameters with prefixes
  * Minor cleanups
  * Fix tests, cleanups
  * Fix order of drop calls
  * Add first bunch of unit tests for list and get / set parameters
  * Clear warnings in rclrs
  * Fix clippy, add set atomically tests
  * Add describe parameter and get parameter types tests
  * Minor cleanups, remove several unwraps
  * Remove commented code
  * Address first round of feedback
  * Allow undeclared parameters in parameter getting services
  * Clippy
  * Run rustfmt
  * Update rclrs/src/parameter/service.rs
  Co-authored-by: jhdcs <48914066+jhdcs@users.noreply.github.com>
  * Change behavior to return NOT_SET for non existing parameters
  * Make use_sim_time parameter read only
  * Format
  * Add a comment to denote why unwrap is safe
  * Use main fmt
  * Add a builder parameter to start parameter services
  ---------
  Co-authored-by: Michael X. Grey <mxgrey@intrinsic.ai>
  Co-authored-by: jhdcs <48914066+jhdcs@users.noreply.github.com>
* Use nightly for style check (`#396 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/396>`_)
  * Use nightly for style check
  * Install nightly for cargo +nightly fmt
  * Fix style in examples
  * Update style for rosidl_runtime_rs
  * Add a comment indicating that nightly release is needed for formatting
  ---------
* Fix Unicode handling in String and WString (`#362 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/362>`_)
  According to https://design.ros2.org/articles/wide_strings.html, the `string` and `wstring` types must use the UTF-8 and UTF-16 encodings (not necessarily enforced), cannot contain null bytes or words (enforced), and, when bounded, are measured in terms of bytes or words.
  Moreover, though the rosidl_runtime_c `U16String` type uses `uint_least16_t`, Rust guarantees the existence of a precise `u16` type, so we should use that instead of `ushort`, which isn't guaranteed to be the same as `uint_least16_t` either. (Rust doesn't support platforms where `uint_least16_t != uint16_t`.)
* Version 0.4.1 (`#353 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/353>`_)
* Update GitHub actions (`#352 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/352>`_)
  * Update GitHub actions
  * Downgrade rust-toolchain to the latest stable version (1.74.0) from 1.80.0
  * Fix issues with latest clippy
  * Fix rustdoc check issue
  * Update Rust toolchain in Dockerfile
* Allow rustdoc to run without a ROS distribution (`#347 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/347>`_)
  * Allow rustdoc to run without a ROS distribution
  * Extensions to get working
  * Fail if generate_docs feature is not enabled and ROS_DISTRO is not set
  * Fix formatting
  * Fix formatting
  * Make clippy slightly less unhappy
  * Do not run clippy for all features if checking rclrs
  * Clean up rcl_bindings.rs
  * Fix comparison
  * Avoid warnings for rosidl_runtime_rs
  * Avoid running cargo test with all features for rclrs
  * Ignore rosidl_runtime_rs for cargo test
  * rosidl_runtime_rs does not have a dyn_msg feature
  * Add comment about the generate_docs feature
  ---------
  Co-authored-by: carter <carterjschultz@gmail.com>
* Version 0.4.0 (`#346 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/346>`_)
* Revert "Version 0.4.0 (`#343 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/343>`_)" (`#344 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/344>`_)
  This reverts commit 8948409e0c6d9ced291b5cffda82036d067bd36d.
* Version 0.4.0 (`#343 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/343>`_)
* - Upgraded bindgen to 0.66.1 (`#323 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/323>`_)
  - Upgraded libloading to 0.8
  - Added instructions to rerun build scripts if dependent files have changed
  - Enabled bindgen to invalidate the built crate whenever any transitively included header files from the wrapper change
  - Removed some default true options from our bindgen builder
  Co-authored-by: Sam Privett <sam@privett.dev>
* Improve efficiency of deserializing strings (`#300 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/300>`_)
* Extend string types (`#293 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/293>`_)
* Remove libc dependencies (`#284 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/284>`_)
* Version 0.3.1 (`#285 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/285>`_)
* Add TYPE_NAME constant to messages and make error fields public (`#277 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/277>`_)
* Format all code with group_imports = StdExternalCrate (`#272 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/272>`_)
* Add README files for rclrs and rosidl_runtime_rs (`#273 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/273>`_)
  These files will be shown by crates.io.
* Bump package versions to 0.3 (`#274 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/274>`_)
* Generalize callbacks for subscriptions (`#260 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/260>`_)
  * Generalize callbacks to subscriptions
  By implementing a trait (SubscriptionCallback) on valid function signatures,
  and making subscriptions accept callbacks that implement this trait, we can now
  * Receive both plain and boxed messages
  * Optionally receive a MessageInfo along with the message, as the second argument
  * Soon, receive a loaned message instead of an owned one
  This corresponds to the functionality in any_subscription_callback.hpp in rclcpp.
* Add support for loaned messages (`#212 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/212>`_)
* Implement Message for Box<T: Message>  (`#235 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/235>`_)
* Fixes for releasing to crates.io (`#231 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/231>`_)
  * Fixups for releasing to crates.io
  * Removed std_msgs as test dependency. Fix rosidl_runtime_rs version
  * Removed test
  * Removed test
* Added support for clients and services (`#146 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/146>`_)
  * Added support for clients and services
* Fix a portability problem in rosidl_runtime_rs::String (`#219 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/219>`_)
  Previously, it was assumed by the $string_conversion_func, for instance, that the output of deref() would be an unsigned integer.
* Add unit testing (`#84 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/84>`_)
  * Copied tests from jhdcs' fork
  * Ran cargo fmt
  * Run clippy on tests as well
  * Make Clippy happy
  * Fix rustfmt warning
  * Added std_msgs to package.xml
  * Disable deref_nullptr warning for generated code
  * Include Soya-Onishi's changes
  * Fix Node::new call
  * Fix spelling mistakes
  Co-authored-by: jhdcs <48914066+jhdcs@users.noreply.github.com>
  * Fix spelling mistakes
  Co-authored-by: jhdcs <48914066+jhdcs@users.noreply.github.com>
  * Fix spelling mistakes
  Co-authored-by: jhdcs <48914066+jhdcs@users.noreply.github.com>
  * Fix formatting
  Co-authored-by: jhdcs <48914066+jhdcs@users.noreply.github.com>
* Add Nikolai and Jacob to authors in Cargo.toml and maintainers in package.xml (`#133 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/133>`_)
* Make all the things Send, and messages Sync as well (`#171 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/171>`_)
  This is required to make multithreading work. For instance, calling publish() on a separate thread doesn't work without these impls.
* Rename RMW-compatible to RMW-native (`#159 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/159>`_)
* Add serde support to messages (`#131 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/131>`_)
* Add comments explaining each dependency (`#122 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/122>`_)
* Removed support for no_std (`#109 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/109>`_)
  * Removed support for no_std
  * Removed more no_std dependencies
  * Removed more no_std dependencies
  * Removed more no_std dependencies
  * Removed downcast
  * Removed TryFrom, not needed with Rust 2021
  * Fix visibility of modules
* Document all public items (`#94 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/94>`_)
* Bump every package to version 0.2 (`#100 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/100>`_)
* Give subscription callback the owned message (`#92 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/92>`_)
  Also remove unused dependency from rosidl_runtime_rs
* Enable Clippy in CI (`#83 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/83>`_)
* Message generation refactoring (`#80 <https://github.com/ros2-rust/rosidl_runtime_rs/issues/80>`_)
  Previously, only messages consisting of basic types and strings were supported. Now, all message types will work, including those that have fields of nested types, bounded types, or arrays.
  Changes:
  - The "rsext" library is deleted
  - Unused messages in "rosidl_generator_rs" are deleted
  - There is a new package, "rosidl_runtime_rs", see below
  - The RMW-compatible messages from C, which do not require an extra conversion step, are exposed in addition to the "idiomatic" messages
  - Publisher and subscription are changed to work with both idiomatic and rmw types, through the unifying `Message` trait
  On `rosidl_runtime_rs`: This package is the successor of `rclrs_msg_utilities` package, but doesn't have much in common with it anymore.
  It provides common types and functionality for messages. The `String` and `Sequence` types and their variants in that package essentially wrap C types from the `rosidl_runtime_c` package and C messages generated by the "rosidl_generator_c" package.
  A number of functions and traits are implemented on these types, so that they feel as ergonomic as possible, for instance, a `seq!` macro for creating a sequence. There is also some documentation and doctests.
  The memory for the (non-pretty) message types is managed by the C allocator.
  Not yet implemented:
  - long double
  - constants
  - Services/clients
  - @verbatim comments
  - ndarray for sequences/arrays of numeric types
  - implementing `Eq`, `Ord` and `Hash` when a message contains no floats
* Contributors: Esteve Fernandez, Grey, Luca Della Vedova, Nathan Wiebe Neufeldt, Nikolai Morin, Oscarchoi, Raghav Mishra, Sam Privett, Tatsuro Sakaguchi

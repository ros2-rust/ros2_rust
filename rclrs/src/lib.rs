#![warn(missing_docs)]
//! Rust client library for ROS 2.
//!
//! Since this library depends on the ROS ecosystem, see the [README][1] for
//! setup instructions.
//!
//! [1]: https://github.com/ros2-rust/ros2_rust/blob/main/README.md
//!
//! This client library is made to be familiar for ROS users who are used to
//! the conventional client libraries `rcl`, `rclcpp`, and `rclpy`, while taking
//! full advantage of the unique strengths of the Rust programming language.
//!
//! The library provides structs that will be familiar to ROS users:
//! - [`Context`]
//! - [`Executor`]
//! - [`Node`]
//! - [`Subscription`]
//! - [`Publisher`]
//! - [`Service`]
//! - [`Client`]
//!
//! It also provides some unique utilities to help leverage Rust language features,
//! such as `async` programming:
//! - [`Worker`]
//! - [`ExecutorCommands`]
//!
//! # Basic Usage
//!
//! To build a typical ROS application, create a [`Context`], followed by an
//! [`Executor`], and then a [`Node`]. Create whatever primitives you need, and
//! then tell the [`Executor`] to spin:
//!
//! ```no_run
//! use rclrs::*;
//! # use crate::rclrs::vendor::example_interfaces;
//!
//! let context = Context::default_from_env()?;
//! let mut executor = context.create_basic_executor();
//! let node = executor.create_node("example_node")?;
//!
//! let subscription = node.create_subscription(
//!     "topic_name",
//!     |msg: example_interfaces::msg::String| {
//!         println!("Received message: {}", msg.data);
//!     }
//! )?;
//!
//! executor.spin(SpinOptions::default()).first_error()?;
//! # Ok::<(), RclrsError>(())
//! ```
//!
//! If your callback needs to interact with some state data, consider using a
//! [`Worker`], especially if that state data needs to be shared with other
//! callbacks:
//!
//! ```no_run
//! # use rclrs::*;
//! #
//! # let context = Context::default_from_env()?;
//! # let mut executor = context.create_basic_executor();
//! # let node = executor.create_node("example_node")?;
//! # use crate::rclrs::vendor::example_interfaces;
//! #
//! // This worker will manage the data for us.
//! // The worker's data is called its payload.
//! let worker = node.create_worker::<Option<String>>(None);
//!
//! // We use the worker to create a subscription.
//! // This subscription's callback can borrow the worker's
//! // payload with its first function argument.
//! let subscription = worker.create_subscription(
//!     "topic_name",
//!     |data: &mut Option<String>, msg: example_interfaces::msg::String| {
//!         // Print out the previous message, if one exists.
//!         if let Some(previous) = data {
//!             println!("Previous message: {}", *previous)
//!         }
//!
//!         // Save the latest message, to be printed out the
//!         // next time this callback is triggered.
//!         *data = Some(msg.data);
//!     }
//! )?;
//!
//! # executor.spin(SpinOptions::default()).first_error()?;
//! # Ok::<(), RclrsError>(())
//! ```
//!
//! # Parameters
//!
//! `rclrs` provides an ergonomic way to declare and use node parameters. A
//! parameter can be declared as [mandatory][crate::MandatoryParameter],
//! [optional][crate::OptionalParameter], or [read-only][crate::ReadOnlyParameter].
//! The API of each reflects their respective constraints.
//! - Mandatory and read-only parameters always have a value that you can [get][MandatoryParameter::get]
//! - Optional parameters will return an [`Option`] when you [get][OptionalParameter::get] from them.
//! - Read-only parameters do not allow you to modify them after they have been declared.
//!
//! The following is a simple example of using a mandatory parameter:
//! ```no_run
//! use rclrs::*;
//! # use crate::rclrs::vendor::example_interfaces;
//! use std::sync::Arc;
//!
//! let mut executor = Context::default_from_env()?.create_basic_executor();
//! let node = executor.create_node("parameter_demo")?;
//!
//! let greeting: MandatoryParameter<Arc<str>> = node
//!     .declare_parameter("greeting")
//!     .default("Hello".into())
//!     .mandatory()?;
//!
//! let _subscription = node.create_subscription(
//!     "greet",
//!     move |msg: example_interfaces::msg::String| {
//!         println!("{}, {}", greeting.get(), msg.data);
//!     }
//! )?;
//!
//! executor.spin(SpinOptions::default()).first_error()?;
//! # Ok::<(), RclrsError>(())
//! ```
//!
//! # Logging
//!
//! `rclrs` provides the same logging utilites as `rclcpp` and `rclpy` with an
//! ergonomic Rust API. [`ToLogParams`] can be used to dictate how logging is
//! performed.
//!
//! ```no_run
//! use rclrs::*;
//! # use crate::rclrs::vendor::example_interfaces;
//! use std::time::Duration;
//!
//! let mut executor = Context::default_from_env()?.create_basic_executor();
//! let node = executor.create_node("logging_demo")?;
//!
//! let _subscription = node.clone().create_subscription(
//!     "logging_demo",
//!     move |msg: example_interfaces::msg::String| {
//!         let data = msg.data;
//!
//!         // You can apply modifiers such as .once() to node.logger()
//!         // to dictate how the logging behaves.
//!         log!(
//!             node.logger().once(),
//!             "First message: {data}",
//!         );
//!
//!         log!(
//!             node.logger().skip_first(),
//!             "Subsequent message: {data}",
//!         );
//!
//!         // You can chain multiple modifiers together.
//!         log_warn!(
//!             node
//!             .logger()
//!             .skip_first()
//!             .throttle(Duration::from_secs(5)),
//!             "Throttled message: {data}",
//!         );
//!     }
//! )?;
//!
//! // Any &str can be used as the logger name and have
//! // logging modifiers applied to it.
//! log_info!(
//!     "notice".once(),
//!     "Ready to begin logging example_interfaces/msg/String messages published to 'logging_demo'.",
//! );
//! log_warn!(
//!     "help",
//!     "Try running\n \
//!     $ ros2 topic pub logging_demo example_interfaces/msg/String \"data: message\"",
//! );
//! executor.spin(SpinOptions::default()).first_error()?;
//! # Ok::<(), RclrsError>(())
//! ```

mod action;
mod arguments;
mod client;
mod clock;
mod context;
mod drop_guard;
mod error;
mod executor;
mod logging;
mod node;
mod parameter;
mod publisher;
mod qos;
mod service;
mod subscription;
mod time;
mod time_source;
mod timer;
pub mod vendor;
mod wait_set;
mod worker;

#[cfg(test)]
mod test_helpers;

mod rcl_bindings;

#[cfg(feature = "dyn_msg")]
pub mod dynamic_message;

pub use action::*;
pub use arguments::*;
pub use client::*;
pub use clock::*;
pub use context::*;
use drop_guard::DropGuard;
pub use error::*;
pub use executor::*;
pub use logging::*;
pub use node::*;
pub use parameter::*;
pub use publisher::*;
pub use qos::*;
pub use rcl_bindings::rmw_request_id_t;
pub use service::*;
pub use subscription::*;
pub use time::*;
use time_source::*;
pub use timer::*;
pub use wait_set::*;
pub use worker::*;

pub use rosidl_runtime_rs::{
    Action as ActionIDL, Message as MessageIDL, RmwMessage as RmwMessageIDL, Service as ServiceIDL,
};

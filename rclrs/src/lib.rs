#![warn(missing_docs)]
//! Rust client library for ROS 2.
//!
//! For getting started, see the [README][1].
//!
//! [1]: https://github.com/ros2-rust/ros2_rust/blob/main/README.md

mod arguments;
mod client;
mod clock;
mod context;
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
mod vendor;
mod wait;

#[cfg(test)]
mod test_helpers;

mod rcl_bindings;

#[cfg(feature = "dyn_msg")]
pub mod dynamic_message;

use std::{sync::Arc, time::Duration};

pub use arguments::*;
pub use client::*;
pub use clock::*;
pub use context::*;
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
pub use wait::*;

/// Polls the node for new messages and executes the corresponding callbacks.
///
/// See [`WaitSet::wait`] for the meaning of the `timeout` parameter.
///
/// This may under some circumstances return
/// [`SubscriptionTakeFailed`][1], [`ClientTakeFailed`][1], [`ServiceTakeFailed`][1] when the wait
/// set spuriously wakes up.
/// This can usually be ignored.
///
/// [1]: crate::RclReturnCode
pub fn spin_once(node: Arc<Node>, timeout: Option<Duration>) -> Result<(), RclrsError> {
    let executor = SingleThreadedExecutor::new();
    executor.add_node(&node)?;
    executor.spin_once(timeout)
}

/// Convenience function for calling [`spin_once`] in a loop.
pub fn spin(node: Arc<Node>) -> Result<(), RclrsError> {
    let executor = SingleThreadedExecutor::new();
    executor.add_node(&node)?;
    executor.spin()
}

/// Creates a new node in the empty namespace.
///
/// Convenience function equivalent to [`Node::new`][1].
/// Please see that function's documentation.
///
/// [1]: crate::Node::new
///
/// # Example
/// ```
/// # use rclrs::{Context, RclrsError};
/// let ctx = Context::new([])?;
/// let node = rclrs::create_node(&ctx, "my_node");
/// assert!(node.is_ok());
/// # Ok::<(), RclrsError>(())
/// ```
pub fn create_node(context: &Context, node_name: &str) -> Result<Arc<Node>, RclrsError> {
    Node::new(context, node_name)
}

/// Creates a [`NodeBuilder`].
///
/// Convenience function equivalent to [`NodeBuilder::new()`][1] and [`Node::builder()`][2].
/// Please see that function's documentation.
///
/// [1]: crate::NodeBuilder::new
/// [2]: crate::Node::builder
///
/// # Example
/// ```
/// # use rclrs::{Context, RclrsError};
/// let context = Context::new([])?;
/// let node_builder = rclrs::create_node_builder(&context, "my_node");
/// let node = node_builder.build()?;
/// assert_eq!(node.name(), "my_node");
/// # Ok::<(), RclrsError>(())
/// ```
pub fn create_node_builder(context: &Context, node_name: &str) -> NodeBuilder {
    Node::builder(context, node_name)
}

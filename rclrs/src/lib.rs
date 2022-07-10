#![warn(missing_docs)]
//! Rust client library for ROS 2.
//!
//! For getting started, see the [README][1].
//!
//! [1]: https://github.com/ros2-rust/ros2_rust/blob/main/README.md

mod context;
mod error;
mod node;
mod parameter;
mod qos;
mod wait;

mod rcl_bindings;

pub use context::*;
pub use error::*;
pub use node::*;
pub use parameter::*;
pub use qos::*;
pub use wait::*;

use rcl_bindings::rcl_context_is_valid;
use std::time::Duration;

pub use rcl_bindings::rmw_request_id_t;

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
pub fn spin_once(node: &Node, timeout: Option<Duration>) -> Result<(), RclrsError> {
    let live_subscriptions = node.live_subscriptions();
    let live_clients = node.live_clients();
    let live_services = node.live_services();
    let ctx = Context {
        rcl_context_mtx: node.rcl_context_mtx.clone(),
    };
    let mut wait_set = WaitSet::new(
        live_subscriptions.len(),
        0,
        0,
        live_clients.len(),
        live_services.len(),
        0,
        &ctx,
    )?;

    for live_subscription in &live_subscriptions {
        wait_set.add_subscription(live_subscription.clone())?;
    }

    for live_client in &live_clients {
        wait_set.add_client(live_client.clone())?;
    }

    for live_service in &live_services {
        wait_set.add_service(live_service.clone())?;
    }

    let ready_entities = wait_set.wait(timeout)?;

    for ready_subscription in ready_entities.subscriptions {
        ready_subscription.execute()?;
    }

    for ready_client in ready_entities.clients {
        ready_client.execute()?;
    }

    for ready_service in ready_entities.services {
        ready_service.execute()?;
    }

    Ok(())
}

/// Convenience function for calling [`spin_once`] in a loop.
///
/// This function additionally checks that the context is still valid.
pub fn spin(node: &Node) -> Result<(), RclrsError> {
    // The context_is_valid functions exists only to abstract away ROS distro differences
    #[cfg(ros_distro = "foxy")]
    // SAFETY: No preconditions for this function.
    let context_is_valid = || unsafe { rcl_context_is_valid(&mut *node.rcl_context_mtx.lock()) };
    #[cfg(not(ros_distro = "foxy"))]
    // SAFETY: No preconditions for this function.
    let context_is_valid = || unsafe { rcl_context_is_valid(&*node.rcl_context_mtx.lock()) };

    while context_is_valid() {
        match spin_once(node, None) {
            Ok(_)
            | Err(RclrsError::RclError {
                code: RclReturnCode::Timeout,
                ..
            }) => (),
            error => return error,
        }
    }
    Ok(())
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
pub fn create_node(context: &Context, node_name: &str) -> Result<Node, RclrsError> {
    Node::builder(context, node_name).build()
}

/// Creates a [`NodeBuilder`][1].
///
/// Convenience function equivalent to [`NodeBuilder::new()`][2] and [`Node::builder()`][3].
/// Please see that function's documentation.
///
/// [1]: crate::NodeBuilder
/// [2]: crate::NodeBuilder::new
/// [3]: crate::Node::builder
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

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};
    use std::time::Duration;

    #[test]
    fn test_spin_once() -> Result<(), RclrsError> {
        let context = Context::new(vec![]).unwrap();
        let mut subscriber_node = create_node(&context, "minimal_subscriber")?;
        let num_messages = Arc::new(Mutex::new(0));
        let received_msg = Arc::new(Mutex::new(String::new()));
        let n = num_messages.clone();
        let m = received_msg.clone();
        let publisher = subscriber_node
            .create_publisher::<std_msgs::msg::String>("topic", QOS_PROFILE_DEFAULT)?;
        let _subscription = subscriber_node.create_subscription::<std_msgs::msg::String, _>(
            "topic",
            QOS_PROFILE_DEFAULT,
            move |msg: std_msgs::msg::String| {
                let mut num_messages = n.lock().unwrap();
                let mut received_msg = m.lock().unwrap();

                *num_messages += 1;
                *received_msg = msg.data;
            },
        )?;

        let message = std_msgs::msg::String {
            data: String::from("Hello World"),
        };
        publisher.publish(message)?;
        spin_once(&subscriber_node, Some(Duration::from_millis(500)))?;

        assert_eq!(*num_messages.lock().unwrap(), 1);
        assert_eq!(&*received_msg.lock().unwrap().as_str(), "Hello World");

        Ok(())
    }
}

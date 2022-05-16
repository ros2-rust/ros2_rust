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

/// Polls the node for new messages and executes the corresponding callbacks.
///
/// See [`WaitSet::wait`] for the meaning of the `timeout` parameter.
///
/// This may under some circumstances return
/// [`SubscriptionTakeFailed`][1] when the wait set spuriously wakes up.
/// This can usually be ignored.
///
/// [1]: crate::RclReturnCode
pub fn spin_once(node: &Node, timeout: Option<Duration>) -> Result<(), RclrsError> {
    let live_subscriptions = node.live_subscriptions();
    let ctx = Context {
        rcl_context_mtx: node.rcl_context_mtx.clone(),
    };
    let mut wait_set = WaitSet::new(live_subscriptions.len(), &ctx)?;

    for live_subscription in &live_subscriptions {
        wait_set.add_subscription(live_subscription.clone())?;
    }

    let ready_entities = wait_set.wait(timeout)?;
    for ready_subscription in ready_entities.subscriptions {
        ready_subscription.execute()?;
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

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};
    use std::time::Duration;

    #[test]
    fn test_spin_once() -> Result<(), RclrsError> {
        let context = Context::new(vec![]).unwrap();
        let mut subscriber_node = context.create_node("minimal_subscriber")?;
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

#![warn(missing_docs)]
//! Rust client library for ROS 2.
//!
//! For getting started, see the [README][1].
//!
//! [1]: https://github.com/ros2-rust/ros2_rust/blob/master/README.md

mod context;
mod error;
mod node;
mod qos;
mod wait;

mod rcl_bindings;

pub use context::*;
pub use error::*;
pub use node::*;
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
/// [1]: crate::SubscriberErrorCode
pub fn spin_once(node: &Node, timeout: Option<Duration>) -> Result<(), RclReturnCode> {
    let live_subscriptions = node.live_subscriptions();
    let ctx = Context {
        handle: node.context.clone(),
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
pub fn spin(node: &Node) -> Result<(), RclReturnCode> {
    // The context_is_valid functions exists only to abstract away ROS distro differences
    #[cfg(ros_distro = "foxy")]
    // SAFETY: No preconditions for this function.
    let context_is_valid = || unsafe { rcl_context_is_valid(&mut *node.context.lock()) };
    #[cfg(not(ros_distro = "foxy"))]
    // SAFETY: No preconditions for this function.
    let context_is_valid = || unsafe { rcl_context_is_valid(&*node.context.lock()) };

    while context_is_valid() {
        if let Some(error) = spin_once(node, None).err() {
            match error {
                RclReturnCode::Timeout => continue,
                error => return Err(error),
            };
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
    fn test_spin_once() -> Result<(), RclReturnCode> {
        let context = Context::new(vec![]).unwrap();
        let mut subscriber_node = context.create_node("minimal_subscriber")?;
        let num_messages = Arc::new(Mutex::new(0));
        let receiver = num_messages.clone();
        let publisher = subscriber_node
            .create_publisher::<std_msgs::msg::String>("topic", QOS_PROFILE_DEFAULT)?;
        let _subscription = subscriber_node.create_subscription::<std_msgs::msg::String, _>(
            "topic",
            QOS_PROFILE_DEFAULT,
            move |_: std_msgs::msg::String| {
                let mut num_messages = receiver.lock().unwrap();
                *num_messages += 1;
            },
        )?;

        let message = std_msgs::msg::String {
            data: String::from("Hello World"),
        };
        publisher.publish(message)?;
        spin_once(&subscriber_node, Some(Duration::from_millis(500)))?;

        assert_eq!(*num_messages.lock().unwrap(), 1);

        Ok(())
    }
}

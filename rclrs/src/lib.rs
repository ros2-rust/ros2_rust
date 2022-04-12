#![no_std]
extern crate alloc;
extern crate core_error;
extern crate downcast;
extern crate rosidl_runtime_rs;

#[cfg(feature = "std")]
extern crate std;

#[cfg(feature = "std")]
extern crate parking_lot;

#[cfg(not(feature = "std"))]
extern crate spin;

pub mod context;
pub mod error;
pub mod node;
pub mod qos;
pub mod wait;

mod rcl_bindings;

pub use self::context::*;
pub use self::error::*;
pub use self::node::*;
pub use self::qos::*;
pub use self::wait::*;

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

/// Convenience function for calling [`Context::spin_once`] in a loop.
///
/// This function additionally checks that the context is still valid.
pub fn spin(node: &Node) -> Result<(), RclReturnCode> {
    // SAFETY: No preconditions for this function.
    while unsafe { rcl_context_is_valid(&mut *node.context.lock() as *mut _) } {
        if let Some(error) = spin_once(node, None).err() {
            match error {
                RclReturnCode::Timeout => continue,
                error => return Err(error),
            };
        }
    }

    Ok(())
}

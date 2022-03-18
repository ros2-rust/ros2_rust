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

use self::rcl_bindings::*;
use core::ops::{Deref, DerefMut};
use wait::{WaitSet, WaitSetErrorResponse};

pub trait Handle<T> {
    type DerefT: Deref<Target = T>;
    type DerefMutT: DerefMut<Target = T>;

    fn get(self) -> Self::DerefT;
    fn get_mut(self) -> Self::DerefMutT;
}

/// Wrapper around [`spin_once`]
pub fn spin<'node>(node: &'node node::Node) -> Result<(), WaitSetErrorResponse> {
    while unsafe { rcl_context_is_valid(&mut *node.context.lock() as *mut _) } {
        if let Some(error) = spin_once(node, 500).err() {
            match error {
                WaitSetErrorResponse::DroppedSubscription
                | WaitSetErrorResponse::ReturnCode(RclReturnCode::Timeout) => continue,
                error => return Err(error),
            };
        }
    }

    Ok(())
}

/// Main function for waiting.
///
/// Following is a schematic representation of the interation of [`spin_once`] with ROS RCL FFI
///
/// +---------------------------------------+
/// |                                       |
/// |   rcl_get_zero_initialized_wait_set   |
/// |                                       |
/// +-----------------+---------------------+
///                   |
///                   |
///        +----------v----------+
///        |  rcl_wait_set_init  |
///        +----------+----------+
///                   |
///        +----------v----------+
///        |  rcl_wait_set_clear |
///        +----------+----------+
///                   |
///   +---------------v------------------+   <---+
///   |rcl_wait_set_add_{subscription,   |       | for all subscriptions,
///   |                  guard_condition |       | services, etc.
///   |                  timer,          |       |
///   |                  client,         |       |
///   |                  service,        |       |
///   |                  e^ent}          |       |
///   +---------------+------------------+ +-----+
///                   |
///         +---------v----------+
///         |     rcl_wait       |
///         +---------+----------+
///                   |
///         +---------v----------+
///         | rcl_wait_set_fini  |
///         +--------------------+
///
///
pub fn spin_once<'node>(node: &'node Node, timeout: i64) -> Result<(), WaitSetErrorResponse> {
    let number_of_subscriptions = node.subscriptions.len();
    let number_of_guard_conditions = 0;
    let number_of_timers = 0;
    let number_of_clients = 0;
    let number_of_services = 0;
    let number_of_events = 0;

    let context = &mut *node.context.lock();

    let mut wait_set = WaitSet::new(
        number_of_subscriptions,
        number_of_guard_conditions,
        number_of_timers,
        number_of_clients,
        number_of_services,
        number_of_events,
        context,
    )?;

    for subscription in &node.subscriptions {
        match wait_set.add_subscription(subscription) {
            Ok(()) => (),
            Err(WaitSetErrorResponse::DroppedSubscription) => (),
            Err(err) => return Err(err),
        };
    }

    wait_set.wait(timeout)?;
    for subscription in &node.subscriptions {
        if let Some(subscription) = subscription.upgrade() {
            subscription.execute()?;
        }
    }

    Ok(())
}

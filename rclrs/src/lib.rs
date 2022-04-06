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

use wait::WaitSet;

/// Wrapper around [`spin_once`]
pub fn spin(node: &node::Node) -> Result<(), RclReturnCode> {
    while unsafe { rcl_context_is_valid(&mut *node.context.lock() as *mut _) } {
        if let Some(error) = spin_once(node, 500).err() {
            match error {
                RclReturnCode::Timeout => continue,
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
pub fn spin_once(node: &Node, timeout_ns: i64) -> Result<(), RclReturnCode> {
    let number_of_subscriptions = node.subscriptions.len();
    let number_of_guard_conditions = 0;
    let number_of_timers = 0;
    let number_of_clients = 0;
    let number_of_services = 0;
    let number_of_events = 0;

    let mut wait_set = WaitSet::new(
        number_of_subscriptions,
        number_of_guard_conditions,
        number_of_timers,
        number_of_clients,
        number_of_services,
        number_of_events,
        &mut *node.context.lock(),
    )?;

    let live_subscriptions = node.live_subscriptions();
    for live_subscription in &live_subscriptions {
        wait_set.add_subscription(&**live_subscription)?;
    }

    wait_set.wait(timeout_ns)?;
    for (i, live_subscription) in live_subscriptions.iter().enumerate() {
        // SAFETY: The `subscriptions` entry is an array of pointers, this dereferencing is
        // equivalent to
        // https://github.com/ros2/rcl/blob/35a31b00a12f259d492bf53c0701003bd7f1745c/rcl/include/rcl/wait.h#L419
        let wait_set_entry = unsafe { *wait_set.handle.subscriptions.add(i) };
        if wait_set_entry.is_null() {
            live_subscription.execute()?;
        }
    }

    Ok(())
}

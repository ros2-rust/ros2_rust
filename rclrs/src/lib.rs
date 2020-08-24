pub mod context;
pub mod error;
pub mod node;
pub mod qos;

mod rcl_bindings;

pub use self::context::*;
pub use self::error::*;
pub use self::node::*;
pub use self::qos::*;

use self::rcl_bindings::*;
use std::ops::{Deref, DerefMut};

pub trait Handle<T> {
    type DerefT: Deref<Target = T>;
    type DerefMutT: DerefMut<Target = T>;

    fn get(self) -> Self::DerefT;
    fn get_mut(self) -> Self::DerefMutT;
}

pub fn spin(node: &Node) -> RclResult {
    while unsafe { rcl_context_is_valid(&mut *node.context.get_mut() as *mut _) } {
        if let Some(error) = spin_once(node, 500).err() {
            match error {
                RclError::Timeout => continue,
                _ => return Err(error),
            }
        }
    }

    Ok(())
}

pub fn spin_once(node: &Node, timeout: i64) -> RclResult {
    let mut wait_set_handle = unsafe { rcl_get_zero_initialized_wait_set() };

    let number_of_subscriptions = node.subscriptions.len();
    let number_of_guard_conditions = 0;
    let number_of_timers = 0;
    let number_of_clients = 0;
    let number_of_services = 0;
    let number_of_events = 0;

    let context = &mut *node.context.get_mut();

    unsafe {
        rcl_wait_set_init(
            &mut wait_set_handle as *mut _,
            number_of_subscriptions,
            number_of_guard_conditions,
            number_of_timers,
            number_of_clients,
            number_of_services,
            number_of_events,
            context,
            rcutils_get_default_allocator(),
        )
        .ok()?;
    }

    unsafe {
        rcl_wait_set_clear(&mut wait_set_handle as *mut _).ok()?;
    }

    for subscription in &node.subscriptions {
        if let Some(subscription) = subscription.upgrade() {
            let subscription_handle = &*subscription.handle().get();
            unsafe {
                rcl_wait_set_add_subscription(
                    &mut wait_set_handle as *mut _,
                    subscription_handle as *const _,
                    std::ptr::null_mut(),
                )
                .ok()?;
            }
        }
    }

    unsafe {
        rcl_wait(&mut wait_set_handle as *mut _, timeout).ok()?;
    }

    for subscription in &node.subscriptions {
        if let Some(subscription) = subscription.upgrade() {
            let mut message = subscription.create_message();
            let result = subscription.take(&mut *message).unwrap();
            if result {
                subscription.callback_fn(message);
            }
        }
    }
    unsafe {
        rcl_wait_set_fini(&mut wait_set_handle as *mut _).ok()?;
    }

    Ok(())
}

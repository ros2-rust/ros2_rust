use std::{any::Any, sync::MutexGuard};

use crate::{rcl_bindings::*, InnerGuardConditionHandle, RclrsError};

/// This provides the public API for executing a waitable item.
pub trait RclPrimitive: Send + Sync {
    /// Trigger this primitive to run.
    ///
    /// * `payload` - The shared payload expected by the primitive. For primitives
    ///   sent through the [`ExecutorChannel`][1], this must be `&mut ()`. For
    ///   primitives sent through a [`WorkerChannel`][2] by a [`Worker`][3] this must be
    ///   the same type as the `Worker`'s generic argument.
    ///
    /// SAFETY: Make sure the type of the payload always matches what the primitive
    /// expects to receive. For now we will return an error if there is a mismatch.
    /// In the future we may use `std::Any::downcast_mut_unchecked` once it
    /// stabilizes, which would give undefined behavior in a mismatch, making it
    /// a serious safety concern.
    ///
    /// [1]: crate::ExecutorChannel
    /// [2]: crate::WorkerChannel
    /// [3]: crate::Worker
    unsafe fn execute(&mut self, payload: &mut dyn Any) -> Result<(), RclrsError>;

    /// Indicate what kind of primitive this is.
    fn kind(&self) -> RclPrimitiveKind;

    /// Provide the handle for this primitive
    fn handle(&self) -> RclPrimitiveHandle;
}

/// Enum to describe the kind of an executable.
#[derive(Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub enum RclPrimitiveKind {
    /// Subscription
    Subscription,
    /// Guard Condition
    GuardCondition,
    /// Timer
    Timer,
    /// Client
    Client,
    /// Service
    Service,
    /// Event
    Event,
}

/// Used by the wait set to obtain the handle of a primitive.
pub enum RclPrimitiveHandle<'a> {
    /// Handle for a subscription
    Subscription(MutexGuard<'a, rcl_subscription_t>),
    /// Handle for a guard condition
    GuardCondition(MutexGuard<'a, InnerGuardConditionHandle>),
    /// Handle for a timer
    Timer(MutexGuard<'a, rcl_timer_t>),
    /// Handle for a client
    Client(MutexGuard<'a, rcl_client_t>),
    /// Handle for a service
    Service(MutexGuard<'a, rcl_service_t>),
    /// Handle for an event
    Event(MutexGuard<'a, rcl_event_t>),
}

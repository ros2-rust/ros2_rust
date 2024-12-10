use std::sync::MutexGuard;

use crate::{rcl_bindings::*, InnerGuardConditionHandle, RclrsError};

/// This provides the public API for executing a waitable item.
pub trait RclPrimitive: Send + Sync {
    /// Trigger this primitive to run.
    fn execute(&mut self) -> Result<(), RclrsError>;

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

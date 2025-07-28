use std::{any::Any, sync::MutexGuard};

use crate::{
    log_error, rcl_bindings::*, InnerGuardConditionHandle, RclrsError, ToResult,
};

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
    unsafe fn execute(&mut self, ready: ReadyKind, payload: &mut dyn Any) -> Result<(), RclrsError>;

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
    /// Action Server
    ActionServer,
}

/// Used by the wait set to obtain the handle of a primitive.
#[derive(Debug)]
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
    /// Handle for an action server
    ActionServer(MutexGuard<'a, rcl_action_server_t>),
}

/// Describe the way in which a waitable is ready.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum ReadyKind {
    /// The basic readiness is for most wait set primitives, which only have one
    /// execution path that may be ready.
    Basic,
    /// This type of readiness is specific to action servers, which consist of
    /// multiple primitives. Any combination of those primitives might be ready,
    /// and we need to know which to execute specifically.
    ActionServer(ActionServerReady),
}

impl ReadyKind {
    /// Convert a pointer's status into a basic ready indicator.
    pub fn from_ptr<T>(ptr: *const T) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self::Basic)
        }
    }

    /// This is used by the basic primitive types to validate that they are
    /// receiving ready information that matches their primitive type.
    pub fn for_basic(self) -> Result<(), RclrsError> {
        match self {
            Self::Basic => Ok(()),
            _ => Err(RclrsError::InvalidReadyInformation {
                expected: Self::Basic,
                received: self,
            })
        }
    }

    pub fn for_action_server(self) -> Result<ActionServerReady, RclrsError> {
        match self {
            Self::ActionServer(ready) => Ok(ready),
            _ => Err(RclrsError::InvalidReadyInformation {
                expected: Self::ActionServer(Default::default()),
                received: self,
            })
        }
    }
}

/// This is the ready information for an action server.
///
/// Action servers provide multiple services bundled together. When a wait set
/// wakes up it is possible for any number of those services to be ready for
/// processing. This struct conveys which of an action's services are ready.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct ActionServerReady {
    /// True if there is a goal request message ready to take, false otherwise.
    pub goal_request: bool,
    /// True if there is a cancel request message ready to take, false otherwise.
    pub cancel_request: bool,
    /// True if there is a result request message ready to take, false otherwise.
    pub result_request: bool,
    //// True if a goal has expired, false otherwise.
    pub goal_expired: bool,
}

impl Default for ActionServerReady {
    fn default() -> Self {
        Self {
            goal_request: false,
            cancel_request: false,
            result_request: false,
            goal_expired: false,
        }
    }
}

impl ActionServerReady {
    /// Check whether any primitives in an action server are ready to be processed.
    ///
    /// SAFETY: This calls a function which is not thread-safe. The wait set and
    /// action server handles must not be in used in any other threads. The
    /// [`MutexGuard`] should ensure this for the action server, but it is up to
    /// the caller to ensure that the wait set is not being simultaneously accessed
    /// in any other thread.
    pub(crate) unsafe fn check(
        wait_set: &rcl_wait_set_t,
        action_server: MutexGuard<rcl_action_server_t>,
    ) -> Option<ReadyKind> {
        let mut ready = ActionServerReady::default();
        let r;
        unsafe {
            // SAFETY: We give a safety warning to ensure that the wait set is
            // not being used elsewhere. The action server handle is guarded by
            // the mutex. With those two requirements met, we do not need to
            // worry about this function not being thread-safe.
            r = rcl_action_server_wait_set_get_entities_ready(
                wait_set,
                &*action_server,
                &mut ready.goal_request,
                &mut ready.cancel_request,
                &mut ready.result_request,
                &mut ready.goal_expired,
            );
        }
        if let Err(err) = r.ok() {
            log_error!(
                "ActionServerReady.check",
                "Error while checking action server: {err}",
            );
        }

        if ready.any_ready() {
            Some(ReadyKind::ActionServer(ready))
        } else {
            None
        }
    }

    /// Check whether any of the primitives of the action server are ready. When
    /// this is false, we can skip producing a [`ReadyKind`] entirely.
    pub fn any_ready(&self) -> bool {
        self.goal_request
        || self.cancel_request
        || self.result_request
        || self.goal_expired
    }
}

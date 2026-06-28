use std::{
    any::Any,
    sync::{Arc, Mutex, MutexGuard},
};

use crate::{
    executor::TimerSchedulerNotify, log_error, rcl_bindings::*, InnerGuardConditionHandle,
    RclrsError, ToResult,
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
    unsafe fn execute(&mut self, ready: ReadyKind, payload: &mut dyn Any)
        -> Result<(), RclrsError>;

    /// Indicate what kind of primitive this is.
    fn kind(&self) -> RclPrimitiveKind;

    /// Provide the handle for this primitive
    fn handle(&self) -> RclPrimitiveHandle<'_>;

    /// Register a push "on ready" callback so an event-driven executor can learn
    /// this primitive has become ready without polling a wait set. The
    /// middleware invokes `on_ready` with the [`ReadyKind`] describing *which*
    /// part of the primitive became ready and the number of new events.
    ///
    /// Most primitives have a single readiness path and call `on_ready` with
    /// [`ReadyKind::Basic`]. Composite primitives (action servers and clients)
    /// register one callback per internal source and call `on_ready` with a
    /// [`ReadyKind::ActionServer`]/[`ReadyKind::ActionClient`] value whose single
    /// matching flag is set, so the executor knows which sub-entity to run.
    ///
    /// Returns `Ok(None)` for primitive kinds that have no rcl push-callback API
    /// (e.g. timers and guard conditions); an event-driven executor drives those
    /// by other means. The returned [`OnReadyHandle`] keeps the callback(s)
    /// registered; dropping it detaches them.
    fn register_on_ready(
        &self,
        on_ready: Box<dyn Fn(ReadyKind, usize) + Send + Sync>,
    ) -> Result<Option<Box<dyn OnReadyHandle>>, RclrsError> {
        // Default: no push-callback support. Suppress the unused parameter.
        let _ = on_ready;
        Ok(None)
    }

    /// For a timer primitive, the handles the Tokio timer scheduler needs to
    /// drive it: the rcl timer and the slot for its notify handle (see
    /// [`TimerSchedulerHandles`]). Timers have no rcl push-callback API, so the
    /// scheduler drives them instead. Returns `None` for every other primitive
    /// kind.
    #[cfg(feature = "tokio-executor")]
    fn timer_scheduler_handles(&self) -> Option<TimerSchedulerHandles> {
        None
    }
}

/// RAII handle that keeps a push "on ready" callback registered with the
/// middleware (see [`RclPrimitive::register_on_ready`]). Dropping it
/// unregisters the callback, before freeing the callback's context, so an
/// event-driven executor can detach an entity simply by dropping this handle.
pub trait OnReadyHandle: Send + Sync {}

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
    /// Action Client
    ActionClient,
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
    /// Handle for an action client
    ActionClient(MutexGuard<'a, rcl_action_client_t>),
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
    /// This type of readiness is specific to action clients, which consist of
    /// multiple primitives. Any combination of those primitives might be ready,
    /// and we need to know which to execute specifically.
    ActionClient(ActionClientReady),
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
            }),
        }
    }

    /// This is used by action servers to get their readiness information.
    pub fn for_action_server(self) -> Result<ActionServerReady, RclrsError> {
        match self {
            Self::ActionServer(ready) => Ok(ready),
            _ => Err(RclrsError::InvalidReadyInformation {
                expected: Self::ActionServer(Default::default()),
                received: self,
            }),
        }
    }

    /// This is used by action clients to get their readiness information.
    pub fn for_action_client(self) -> Result<ActionClientReady, RclrsError> {
        match self {
            Self::ActionClient(ready) => Ok(ready),
            _ => Err(RclrsError::InvalidReadyInformation {
                expected: Self::ActionClient(Default::default()),
                received: self,
            }),
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
    /// True if a goal has expired, false otherwise.
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
        let r = unsafe {
            // SAFETY: We give a safety warning to ensure that the wait set is
            // not being used elsewhere. The action server handle is guarded by
            // the mutex. With those two requirements met, we do not need to
            // worry about this function not being thread-safe.
            rcl_action_server_wait_set_get_entities_ready(
                wait_set,
                &*action_server,
                &mut ready.goal_request,
                &mut ready.cancel_request,
                &mut ready.result_request,
                &mut ready.goal_expired,
            )
        };

        if let Err(err) = r.ok() {
            log_error!(
                "ActionServerReady.check",
                "Error while checking readiness for action server: {err}",
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
        self.goal_request || self.cancel_request || self.result_request || self.goal_expired
    }
}

/// This is the ready information for an action client.
///
/// Action clients contain multiple service clients bundled together, as well as
/// some subscribers. When a wait set wakes up it is possible for any number of
/// those services or subscriptions to be ready for processing. This struct
/// conveys which of an action client's messages are ready.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct ActionClientReady {
    /// True if there is a feedback message ready to take, false otherwise.
    pub feedback: bool,
    /// True if there is a status message ready to take, false otherwise.
    pub status: bool,
    /// True if there is a goal response message ready to take, false otherwise.
    pub goal_response: bool,
    /// True if there is a cancel response message ready to take, false otherwise.
    pub cancel_response: bool,
    /// True if there is a result response message ready to take, false otherwise.
    pub result_response: bool,
}

impl ActionClientReady {
    pub(crate) unsafe fn check(
        wait_set: &rcl_wait_set_t,
        action_client: MutexGuard<rcl_action_client_t>,
    ) -> Option<ReadyKind> {
        let mut ready = ActionClientReady::default();
        let r = unsafe {
            // SAFETY: We give a safety warning to ensure that the wait set is
            // not being used elsewhere. The action client handle is guarded by
            // the mutex. With those two requirements met, we do not need to
            // worry about this function not being thread-safe.
            rcl_action_client_wait_set_get_entities_ready(
                wait_set,
                &*action_client,
                &mut ready.feedback,
                &mut ready.status,
                &mut ready.goal_response,
                &mut ready.cancel_response,
                &mut ready.result_response,
            )
        };

        if let Err(err) = r.ok() {
            log_error!(
                "ActionClientReady.check",
                "Error while checking readiness for action client: {err}",
            );
        }

        if ready.any_ready() {
            Some(ReadyKind::ActionClient(ready))
        } else {
            None
        }
    }

    /// Check whether any of the primitives of the action client are ready. When
    /// this is false, we can skip producing a [`ReadyKind`] entirely.
    pub fn any_ready(&self) -> bool {
        self.feedback
            || self.status
            || self.goal_response
            || self.cancel_response
            || self.result_response
    }
}

impl Default for ActionClientReady {
    fn default() -> Self {
        Self {
            feedback: false,
            status: false,
            goal_response: false,
            cancel_response: false,
            result_response: false,
        }
    }
}

/// The handles the Tokio timer scheduler needs to drive a timer: the rcl timer
/// (to read deadlines from) and the slot where the timer keeps the
/// [`TimerSchedulerNotify`] it receives at registration, so its `reset`/`cancel`
/// and drop can reach the scheduler. Returned by
/// [`RclPrimitive::timer_scheduler_handles`]; `None` for non-timer primitives.
#[cfg(feature = "tokio-executor")]
pub(crate) struct TimerSchedulerHandles {
    /// The rcl timer the scheduler reads deadlines from.
    pub rcl_timer: Arc<Mutex<rcl_timer_t>>,
    /// Slot where the timer stores the [`TimerSchedulerNotify`] it gets back from
    /// registration, so reset/cancel/drop can notify the scheduler.
    pub notify_slot: Arc<Mutex<Option<TimerSchedulerNotify>>>,
}

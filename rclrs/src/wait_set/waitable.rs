use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};

use crate::{
    error::ToResult, log_error, rcl_bindings::*, ActionClientReady, ActionServerReady,
    GuardCondition, RclPrimitive, RclPrimitiveHandle, RclPrimitiveKind, RclrsError, ReadyKind,
};

/// This struct manages the presence of an rcl primitive inside the wait set.
/// It will keep track of where the primitive is within the wait set as well as
/// automatically remove the primitive from the wait set once it isn't being
/// used anymore.
#[must_use = "If you do not give the Waitable to a WaitSet then it will never be useful"]
pub struct Waitable {
    pub(super) primitive: Box<dyn RclPrimitive>,
    in_use: Arc<AtomicBool>,
    index_in_wait_set: Option<usize>,
}

impl Waitable {
    /// Create a new waitable.
    pub fn new(
        primitive: Box<dyn RclPrimitive>,
        guard_condition: Option<Arc<GuardCondition>>,
    ) -> (Self, WaitableLifecycle) {
        let in_use = Arc::new(AtomicBool::new(true));
        let waitable = Self {
            primitive,
            in_use: Arc::clone(&in_use),
            index_in_wait_set: None,
        };

        let lifecycle = WaitableLifecycle {
            in_use,
            guard_condition,
        };
        (waitable, lifecycle)
    }

    pub(super) fn in_wait_set(&self) -> bool {
        self.index_in_wait_set.is_some()
    }

    pub(super) fn in_use(&self) -> bool {
        self.in_use.load(Ordering::Relaxed)
    }

    pub(super) fn is_ready(&self, wait_set: &rcl_wait_set_t) -> Option<ReadyKind> {
        match self.primitive.kind() {
            RclPrimitiveKind::Subscription => {
                self.index_in_wait_set.and_then(|index| {
                    // SAFETY: Each field in the wait set is an array of points.
                    // The dereferencing that we do is equivalent to obtaining the
                    // element of the array at the index-th position.
                    ReadyKind::from_ptr(unsafe { wait_set.subscriptions.add(index) })
                })
            }
            RclPrimitiveKind::GuardCondition => {
                self.index_in_wait_set.and_then(|index| {
                    // SAFETY: Each field in the wait set is an array of points.
                    // The dereferencing that we do is equivalent to obtaining the
                    // element of the array at the index-th position.
                    ReadyKind::from_ptr(unsafe { wait_set.guard_conditions.add(index) })
                })
            }
            RclPrimitiveKind::Service => {
                self.index_in_wait_set.and_then(|index| {
                    // SAFETY: Each field in the wait set is an array of points.
                    // The dereferencing that we do is equivalent to obtaining the
                    // element of the array at the index-th position.
                    ReadyKind::from_ptr(unsafe { wait_set.services.add(index) })
                })
            }
            RclPrimitiveKind::Client => {
                self.index_in_wait_set.and_then(|index| {
                    // SAFETY: Each field in the wait set is an array of points.
                    // The dereferencing that we do is equivalent to obtaining the
                    // element of the array at the index-th position.
                    ReadyKind::from_ptr(unsafe { wait_set.clients.add(index) })
                })
            }
            RclPrimitiveKind::Timer => {
                self.index_in_wait_set.and_then(|index| {
                    // SAFETY: Each field in the wait set is an array of points.
                    // The dereferencing that we do is equivalent to obtaining the
                    // element of the array at the index-th position.
                    ReadyKind::from_ptr(unsafe { wait_set.timers.add(index) })
                })
            }
            RclPrimitiveKind::Event => {
                self.index_in_wait_set.and_then(|index| {
                    // SAFETY: Each field in the wait set is an array of points.
                    // The dereferencing that we do is equivalent to obtaining the
                    // element of the array at the index-th position.
                    ReadyKind::from_ptr(unsafe { wait_set.events.add(index) })
                })
            }
            RclPrimitiveKind::ActionServer => {
                match self.primitive.handle() {
                    RclPrimitiveHandle::ActionServer(handle) => {
                        // SAFETY: We have exclusive ownership of the wait set
                        // and the action server handle right now, which satisfies
                        // the safety requirements of the function.
                        unsafe { ActionServerReady::check(wait_set, handle) }
                    }
                    handle => {
                        log_error!(
                            "waitable.is_ready",
                            "Invalid handle for ActionServer type: {handle:?}. \
                            This indicates a bug in the implementation of rclrs. \
                            Please report this to the rclrs maintainers.",
                        );
                        None
                    }
                }
            }
            RclPrimitiveKind::ActionClient => {
                match self.primitive.handle() {
                    RclPrimitiveHandle::ActionClient(handle) => {
                        // SAFETY: We have exclusive ownership of the wait set
                        // and the action client handle right now, which satisfies
                        // the safety requirements of the function.
                        unsafe { ActionClientReady::check(wait_set, handle) }
                    }
                    handle => {
                        log_error!(
                            "waitable.is_ready",
                            "Invalid handle for ActionClient type: {handle:?}. \
                            This indicates a bug in the implementation of rclrs. \
                            Please report this to the rclrs maintainers.",
                        );
                        None
                    }
                }
            }
        }
    }

    pub(super) fn add_to_wait_set(
        &mut self,
        wait_set: &mut rcl_wait_set_t,
    ) -> Result<(), RclrsError> {
        let mut index = 0;
        unsafe {
            // SAFETY: The Executable is responsible for maintaining the lifecycle
            // of the handle, so it is guaranteed to be valid here.
            match self.primitive.handle() {
                RclPrimitiveHandle::Subscription(handle) => {
                    rcl_wait_set_add_subscription(wait_set, &*handle, &mut index)
                }
                RclPrimitiveHandle::GuardCondition(handle) => handle.use_handle(|handle| {
                    rcl_wait_set_add_guard_condition(wait_set, handle, &mut index)
                }),
                RclPrimitiveHandle::Service(handle) => {
                    rcl_wait_set_add_service(wait_set, &*handle, &mut index)
                }
                RclPrimitiveHandle::Client(handle) => {
                    rcl_wait_set_add_client(wait_set, &*handle, &mut index)
                }
                RclPrimitiveHandle::Timer(handle) => {
                    rcl_wait_set_add_timer(wait_set, &*handle, &mut index)
                }
                RclPrimitiveHandle::Event(handle) => {
                    rcl_wait_set_add_event(wait_set, &*handle, &mut index)
                }
                RclPrimitiveHandle::ActionServer(handle) => {
                    rcl_action_wait_set_add_action_server(wait_set, &*handle, std::ptr::null_mut())
                }
                RclPrimitiveHandle::ActionClient(handle) => rcl_action_wait_set_add_action_client(
                    wait_set,
                    &*handle,
                    std::ptr::null_mut(),
                    std::ptr::null_mut(),
                ),
            }
        }
        .ok()?;

        self.index_in_wait_set = Some(index);
        Ok(())
    }
}

/// This is used internally to track whether an rcl primitive is still being
/// used. When this gets dropped, the rcl primitive will automatically be
/// removed from the wait set.
#[must_use = "If you do not hold onto the WaiterLifecycle, then its Waiter will be immediately dropped"]
pub struct WaitableLifecycle {
    in_use: Arc<AtomicBool>,
    guard_condition: Option<Arc<GuardCondition>>,
}

impl Drop for WaitableLifecycle {
    fn drop(&mut self) {
        self.in_use.store(false, Ordering::Release);
        if let Some(guard_condition) = &self.guard_condition {
            guard_condition.trigger().ok();
        }
    }
}

/// Count the number of rcl primitives in the wait set.
#[derive(Default, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct WaitableCount {
    /// How many subscriptions are present
    pub subscriptions: usize,
    /// How many guard conditions are present
    pub guard_conditions: usize,
    /// How many timers are present
    pub timers: usize,
    /// How many clients are present
    pub clients: usize,
    /// How many services are present
    pub services: usize,
    /// How many events are present
    pub events: usize,
}

impl WaitableCount {
    /// Begin a new count with everything starting out at zero.
    pub fn new() -> Self {
        Self::default()
    }

    pub(super) fn add_group(&mut self, kind: &RclPrimitiveKind, waitables: &Vec<Waitable>) {
        match kind {
            RclPrimitiveKind::Subscription => self.subscriptions += waitables.len(),
            RclPrimitiveKind::GuardCondition => self.guard_conditions += waitables.len(),
            RclPrimitiveKind::Timer => self.timers += waitables.len(),
            RclPrimitiveKind::Client => self.clients += waitables.len(),
            RclPrimitiveKind::Service => self.services += waitables.len(),
            RclPrimitiveKind::Event => self.events += waitables.len(),
            RclPrimitiveKind::ActionServer => {
                for waitable in waitables {
                    self.add_single(&*waitable.primitive);
                }
            }
            RclPrimitiveKind::ActionClient => {
                for waitable in waitables {
                    self.add_single(&*waitable.primitive);
                }
            }
        }
    }

    fn add_single(&mut self, primitive: &dyn RclPrimitive) {
        match primitive.handle() {
            RclPrimitiveHandle::Subscription(_) => self.subscriptions += 1,
            RclPrimitiveHandle::GuardCondition(_) => self.guard_conditions += 1,
            RclPrimitiveHandle::Timer(_) => self.timers += 1,
            RclPrimitiveHandle::Client(_) => self.clients += 1,
            RclPrimitiveHandle::Service(_) => self.services += 1,
            RclPrimitiveHandle::Event(_) => self.events += 1,
            RclPrimitiveHandle::ActionServer(handle) => {
                let mut count = WaitableCount::new();
                let r = unsafe {
                    // SAFETY: The handle is kept safe by the mutex guard, and
                    // there are no other preconditions.
                    rcl_action_server_wait_set_get_num_entities(
                        &*handle,
                        &mut count.subscriptions,
                        &mut count.guard_conditions,
                        &mut count.timers,
                        &mut count.clients,
                        &mut count.services,
                    )
                };
                if let Err(err) = r.ok() {
                    log_error!(
                        "waitable_count.add_single",
                        "Error occurred while counting primitives for an action server. \
                        This should not happen, please report it to the rclrs maintainers: {err}",
                    );
                }

                *self += count;
            }
            RclPrimitiveHandle::ActionClient(handle) => {
                let mut count = WaitableCount::new();
                let r = unsafe {
                    // SAFETY: The handle is kept safe by the mutex guard, and
                    // there are no other preconditions.
                    rcl_action_client_wait_set_get_num_entities(
                        &*handle,
                        &mut count.subscriptions,
                        &mut count.guard_conditions,
                        &mut count.timers,
                        &mut count.clients,
                        &mut count.services,
                    )
                };
                if let Err(err) = r.ok() {
                    log_error!(
                        "waitable_count.add_single",
                        "Error occurred while counting primitives for an action client. \
                        This should not happen, please report it to the rclrs maintainers: {err}",
                    );
                }

                *self += count;
            }
        }
    }

    pub(super) unsafe fn initialize(
        &self,
        rcl_context: &mut rcl_context_s,
    ) -> Result<rcl_wait_set_s, RclrsError> {
        unsafe {
            // SAFETY: Getting a zero-initialized value is always safe
            let mut rcl_wait_set = rcl_get_zero_initialized_wait_set();
            // SAFETY: We're passing in a zero-initialized wait set and a valid context.
            // There are no other preconditions.
            rcl_wait_set_init(
                &mut rcl_wait_set,
                self.subscriptions,
                self.guard_conditions,
                self.timers,
                self.clients,
                self.services,
                self.events,
                &mut *rcl_context,
                rcutils_get_default_allocator(),
            )
            .ok()?;
            Ok(rcl_wait_set)
        }
    }

    pub(super) unsafe fn resize(
        &self,
        rcl_wait_set: &mut rcl_wait_set_t,
    ) -> Result<(), RclrsError> {
        unsafe {
            rcl_wait_set_resize(
                rcl_wait_set,
                self.subscriptions,
                self.guard_conditions,
                self.timers,
                self.clients,
                self.services,
                self.events,
            )
        }
        .ok()
    }
}

impl std::ops::Add for WaitableCount {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Self {
            subscriptions: self.subscriptions + rhs.subscriptions,
            guard_conditions: self.guard_conditions + rhs.guard_conditions,
            timers: self.timers + rhs.timers,
            clients: self.clients + rhs.clients,
            services: self.services + rhs.services,
            events: self.events + rhs.events,
        }
    }
}

impl std::ops::AddAssign for WaitableCount {
    fn add_assign(&mut self, rhs: Self) {
        let count = *self + rhs;
        *self = count;
    }
}

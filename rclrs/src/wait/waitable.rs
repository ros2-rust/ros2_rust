use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc, MutexGuard,
};

use crate::{
    error::ToResult,
    rcl_bindings::*,
    RclrsError, GuardCondition,
};

/// Enum to describe the kind of an executable.
#[derive(Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub enum ExecutableKind {
    Subscription,
    GuardCondition,
    Timer,
    Client,
    Service,
    Event,
}

/// Used by the wait set to obtain the handle of an executable.
pub enum ExecutableHandle<'a> {
    Subscription(MutexGuard<'a, rcl_subscription_t>),
    GuardCondition(MutexGuard<'a, rcl_guard_condition_t>),
    Timer(MutexGuard<'a, rcl_timer_t>),
    Client(MutexGuard<'a, rcl_client_t>),
    Service(MutexGuard<'a, rcl_service_t>),
    Event(MutexGuard<'a, rcl_event_t>),
}

#[derive(Default, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct WaitableCount {
    pub subscriptions: usize,
    pub guard_conditions: usize,
    pub timers: usize,
    pub clients: usize,
    pub services: usize,
    pub events: usize,
}

impl WaitableCount {
    pub fn new() -> Self {
        Self::default()
    }

    pub(super) fn add(&mut self, kind: ExecutableKind, count: usize) {
        match kind {
            ExecutableKind::Subscription => self.subscriptions += count,
            ExecutableKind::GuardCondition => self.guard_conditions += count,
            ExecutableKind::Timer => self.timers += count,
            ExecutableKind::Client => self.clients += count,
            ExecutableKind::Service => self.services += count,
            ExecutableKind::Event => self.events += count,
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

/// This provides the public API for executing a waitable item.
pub trait Executable {
    /// Trigger this executable to run.
    fn execute(&mut self) -> Result<(), RclrsError>;

    /// Indicate what kind of executable this is.
    fn kind(&self) -> ExecutableKind;

    /// Provide the handle for this executable
    fn handle(&self) -> ExecutableHandle;
}

#[must_use = "If you do not give the Waiter to a WaitSet then it will never be useful"]
pub struct Waitable {
    pub(super) executable: Box<dyn Executable + Send + Sync>,
    in_use: Arc<AtomicBool>,
    index_in_wait_set: Option<usize>,
}

impl Waitable {
    pub fn new(
        waitable: Box<dyn Executable + Send + Sync>,
        guard_condition: Option<Arc<GuardCondition>>,
    ) -> (Self, WaitableLifecycle) {
        let in_use = Arc::new(AtomicBool::new(true));
        let waiter = Self {
            executable: waitable,
            in_use: Arc::clone(&in_use),
            index_in_wait_set: None,
        };

        let lifecycle = WaitableLifecycle { in_use, guard_condition };
        (waiter, lifecycle)
    }

    pub(super) fn in_wait_set(&self) -> bool {
        self.index_in_wait_set.is_some()
    }

    pub(super) fn in_use(&self) -> bool {
        self.in_use.load(Ordering::Relaxed)
    }

    pub(super) fn is_ready(&self, wait_set: &rcl_wait_set_t) -> bool {
        self.index_in_wait_set.is_some_and(|index| {
            let ptr_is_null = unsafe {
                // SAFETY: Each field in the wait set is an array of points.
                // The dereferencing that we do is equivalent to obtaining the
                // element of the array at the index-th position.
                match self.executable.kind() {
                    ExecutableKind::Subscription => wait_set.subscriptions.add(index).is_null(),
                    ExecutableKind::GuardCondition => wait_set.guard_conditions.add(index).is_null(),
                    ExecutableKind::Service => wait_set.services.add(index).is_null(),
                    ExecutableKind::Client => wait_set.clients.add(index).is_null(),
                    ExecutableKind::Timer => wait_set.timers.add(index).is_null(),
                    ExecutableKind::Event => wait_set.events.add(index).is_null(),
                }
            };
            !ptr_is_null
        })
    }

    pub(super) fn add_to_wait_set(
        &mut self,
        wait_set: &mut rcl_wait_set_t,
    ) -> Result<(), RclrsError> {

        let mut index = 0;
        unsafe {
            // SAFETY: The Executable is responsible for maintaining the lifecycle
            // of the handle, so it is guaranteed to be valid here.
            match self.executable.handle() {
                ExecutableHandle::Subscription(handle) => {
                    rcl_wait_set_add_subscription(wait_set, &*handle, &mut index)
                }
                ExecutableHandle::GuardCondition(handle) => {
                    rcl_wait_set_add_guard_condition(wait_set, &*handle, &mut index)
                }
                ExecutableHandle::Service(handle) => {
                    rcl_wait_set_add_service(wait_set, &*handle, &mut index)
                }
                ExecutableHandle::Client(handle) => {
                    rcl_wait_set_add_client(wait_set, &*handle, &mut index)
                }
                ExecutableHandle::Timer(handle) => {
                    rcl_wait_set_add_timer(wait_set, &*handle, &mut index)
                }
                ExecutableHandle::Event(handle) => {
                    rcl_wait_set_add_event(wait_set, &*handle, &mut index)
                }
            }
        }
        .ok()?;

        self.index_in_wait_set = Some(index);
        Ok(())
    }
}

#[must_use = "If you do not hold onto the WaiterLifecycle, then its Waiter will be immediately dropped"]
pub struct WaitableLifecycle {
    in_use: Arc<AtomicBool>,
    guard_condition: Option<Arc<GuardCondition>>,
}

impl Drop for WaitableLifecycle {
    fn drop(&mut self) {
        self.in_use.store(false, Ordering::Release);
        if let Some(guard_condition) = &self.guard_condition {
            guard_condition.trigger();
        }
    }
}

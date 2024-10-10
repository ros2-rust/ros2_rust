use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};

use crate::{
    rcl_bindings::*,
    RclrsError, WaitSet,
};

#[derive(Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum WaitableKind {
    Subscription,
    GuardCondition,
    Timer,
    Client,
    Service,
    Event,
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

    pub(super) fn add(&mut self, kind: WaitableKind, count: usize) {
        match kind {
            WaitableKind::Subscription => self.subscriptions += count,
            WaitableKind::GuardCondition => self.guard_conditions += count,
            WaitableKind::Timer => self.timers += count,
            WaitableKind::Client => self.clients += count,
            WaitableKind::Service => self.services += count,
            WaitableKind::Event => self.events += count,
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
    /// Indicate what kind of executable this is.
    fn kind(&self) -> WaitableKind;

    /// Trigger this executable to run.
    fn execute(&mut self) -> Result<(), RclrsError>;
}

/// This provides the internal APIs for a waitable item to interact with the
/// wait set that manages it.
pub(crate) trait Waitable: Executable {
    /// Add this to a wait set
    unsafe fn add_to_wait_set(
        &mut self,
        wait_set: &mut rcl_wait_set_t,
    ) -> Result<usize, RclrsError>;

    unsafe fn is_ready(
        &self,
        wait_set: &rcl_wait_set_t,
        index: usize,
    ) -> bool;
}

pub struct Waiter {
    pub(super) waitable: Box<dyn Waitable>,
    in_use: Arc<AtomicBool>,
    index_in_wait_set: Option<usize>,
}

impl Waiter {
    pub fn new(waitable: Box<dyn Waitable>) -> (Self, WaiterLifecycle) {
        let in_use = Arc::new(AtomicBool::new(true));
        let waiter = Self {
            waitable,
            in_use: Arc::clone(&in_use),
            index_in_wait_set: None,
        };

        let lifecycle = WaiterLifecycle { in_use };
        (waiter, lifecycle)
    }

    pub(super) fn in_wait_set(&self) -> bool {
        self.index_in_wait_set.is_some()
    }

    pub(super) fn in_use(&self) -> bool {
        self.in_use.load(Ordering::Relaxed)
    }

    pub(super) fn is_ready(&self, wait_set: &rcl_wait_set_t) -> bool {
        self.index_in_wait_set.is_some_and(|index|
            unsafe {
                // SAFETY: The Waitable::is_ready function is marked as unsafe
                // because this is the only place that it makes sense to use it.
                self.waitable.is_ready(wait_set, index)
            }
        )
    }

    pub(super) fn add_to_wait_set(
        &mut self,
        wait_set: &mut WaitSet,
    ) -> Result<(), RclrsError> {
        self.index_in_wait_set = Some(
            unsafe {
                // SAFETY: The Waitable::add_to_wait_set function is marked as
                // unsafe because this is the only place that it makes sense to use it.
                self.waitable.add_to_wait_set(wait_set)?
            }
        );
        Ok(())
    }
}

pub struct WaiterLifecycle {
    in_use: Arc<AtomicBool>,
}

impl Drop for WaiterLifecycle {
    fn drop(&mut self) {
        self.in_use.store(false, Ordering::Relaxed);
    }
}

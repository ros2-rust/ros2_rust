use crate::{
    rcl_bindings::*,
    RclrsError,
};

#[derive(Clone, Copy)]
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

pub trait Executable {
    fn execute(&mut self) -> Result<(), RclrsError>;
}

pub(crate) trait Waitable: Executable {
    unsafe fn add_to_wait_set(
        &mut self,
        wait_set: &mut rcl_wait_set_t,
    ) -> Result<usize, RclrsError>;

    fn kind(&self) -> WaitableKind;
}

pub struct Waiter {
    waitable: Box<dyn Waitable>,
    index_in_wait_set: Option<usize>,
}

impl Waiter {
    pub fn new<T: Waitable>(waitable: T) -> Self {
        Self {
            waitable: Box::new(waitable),
            index_in_wait_set: None,
        }
    }

}

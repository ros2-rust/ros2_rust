use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};

use crate::{
    error::ToResult, rcl_bindings::*, GuardCondition, RclPrimitive, RclPrimitiveHandle,
    RclPrimitiveKind, RclrsError,
};

/// This struct manages the presence of an rcl primitive inside the wait set.
/// It will keep track of where the primitive is within the wait set as well as
/// automatically remove the primitive from the wait set once it isn't being
/// used anymore.
#[must_use = "If you do not give the Waiter to a WaitSet then it will never be useful"]
pub struct Waitable {
    pub(super) primitive: Box<dyn RclPrimitive + Send + Sync>,
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
        let waiter = Self {
            primitive,
            in_use: Arc::clone(&in_use),
            index_in_wait_set: None,
        };

        let lifecycle = WaitableLifecycle {
            in_use,
            guard_condition,
        };
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
                match self.primitive.kind() {
                    RclPrimitiveKind::Subscription => wait_set.subscriptions.add(index).is_null(),
                    RclPrimitiveKind::GuardCondition => {
                        wait_set.guard_conditions.add(index).is_null()
                    }
                    RclPrimitiveKind::Service => wait_set.services.add(index).is_null(),
                    RclPrimitiveKind::Client => wait_set.clients.add(index).is_null(),
                    RclPrimitiveKind::Timer => wait_set.timers.add(index).is_null(),
                    RclPrimitiveKind::Event => wait_set.events.add(index).is_null(),
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
            match self.primitive.handle() {
                RclPrimitiveHandle::Subscription(handle) => {
                    rcl_wait_set_add_subscription(wait_set, &*handle, &mut index)
                }
                RclPrimitiveHandle::GuardCondition(handle) => handle.use_handle(|handle| {
                    rcl_wait_set_add_guard_condition(wait_set, &*handle, &mut index)
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

    pub(super) fn add(&mut self, kind: RclPrimitiveKind, count: usize) {
        match kind {
            RclPrimitiveKind::Subscription => self.subscriptions += count,
            RclPrimitiveKind::GuardCondition => self.guard_conditions += count,
            RclPrimitiveKind::Timer => self.timers += count,
            RclPrimitiveKind::Client => self.clients += count,
            RclPrimitiveKind::Service => self.services += count,
            RclPrimitiveKind::Event => self.events += count,
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

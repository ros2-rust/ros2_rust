//! Safe RAII wrapper around rcl's "on new ___" push-callback APIs
//! (`rcl_subscription_set_on_new_message_callback` and the service/client/event
//! equivalents).
//!
//! These let an event-driven executor learn that an entity has become ready
//! *without* polling `rcl_wait`: the middleware invokes a C callback (possibly
//! from its own thread) when data arrives. We forward that to a Rust closure,
//! which an executor uses to enqueue work.
//!
//! [`OnReadyRegistration`] is generic over the entity handle type `H`. Each
//! entity module provides a [`SetOnReadyFn`] that locks its handle and calls the
//! appropriate `rcl_*_set_on_new_*_callback`; the registration owns the boxed
//! callback context and the handle, and deregisters on drop.
//!
//! # Safety model
//!
//! rcl stores the `user_data` pointer we hand it and passes it back to the C
//! callback on every event. That pointer must stay valid for as long as the
//! callback is registered. We therefore:
//!
//! - box the [`EventCallbackCtx`] so it has a stable heap address, and
//! - in `Drop`, **unregister the callback first** (so the middleware can no
//!   longer invoke the trampoline) and only then free the context.
//!
//! Getting that ordering wrong is a use-after-free, since the middleware may be
//! calling the trampoline from another thread at the moment of teardown.
//!
//! # Panic containment
//!
//! The trampoline is an `extern "C"` entry point. A Rust panic must not unwind
//! across that boundary. User closures are therefore invoked inside
//! [`std::panic::catch_unwind`]; on panic the registration is disabled and a
//! fatal log is emitted once. An optional [`OnReadyPanicHook`] lets an executor
//! record the failure and wake its spin driver without panicking or allocating
//! in the reporter itself.

use std::{
    os::raw::c_void,
    panic::AssertUnwindSafe,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
};

use crate::{log_fatal, rcl_bindings::*, OnReadyHandle, RclrsError, ToResult};

/// Optional hook invoked once when a push callback panics. Must not panic or
/// allocate unboundedly; an executor uses this to record a fatal spin error and
/// wake its driver.
pub(crate) type OnReadyPanicHook = Arc<dyn Fn() + Send + Sync>;

/// The context carried through rcl as `user_data`. Boxed so its address is
/// stable for the lifetime of the registration.
struct EventCallbackCtx {
    /// When false, the trampoline returns without invoking the closure.
    enabled: AtomicBool,
    /// Set after the first panic so we log and notify at most once.
    panic_reported: AtomicBool,
    on_ready: Box<dyn Fn(usize) + Send + Sync>,
    on_panic: Option<OnReadyPanicHook>,
}

impl EventCallbackCtx {
    fn new(on_ready: Box<dyn Fn(usize) + Send + Sync>, on_panic: Option<OnReadyPanicHook>) -> Self {
        Self {
            enabled: AtomicBool::new(true),
            panic_reported: AtomicBool::new(false),
            on_ready,
            on_panic,
        }
    }

    /// Record the first panic, disable this registration, and notify an executor
    /// if one was wired in. Must not panic or allocate unboundedly.
    fn report_panic(&self) {
        if self.panic_reported.swap(true, Ordering::AcqRel) {
            return;
        }
        self.enabled.store(false, Ordering::Release);
        log_fatal!(
            "rclrs.executor.event_callback",
            "A push on-ready callback panicked; the registration has been disabled \
             and will not be invoked again.",
        );
        if let Some(on_panic) = &self.on_panic {
            on_panic();
        }
    }
}

/// The C trampoline that rcl/rmw invokes when an entity becomes ready. It may be
/// called from a middleware thread, so it does nothing but forward to the Rust
/// closure. It must not run user code or take locks that could deadlock the
/// middleware, and it must not let a Rust panic unwind across the C ABI.
unsafe extern "C" fn on_ready_trampoline(user_data: *const c_void, number_of_events: usize) {
    // SAFETY: `user_data` is the pointer to the `EventCallbackCtx` we passed to
    // the rcl setter. It stays valid until the owning registration's `Drop`
    // clears the callback, which always happens before the box is freed.
    let ctx = unsafe { &*(user_data as *const EventCallbackCtx) };
    if !ctx.enabled.load(Ordering::Acquire) {
        return;
    }

    let panicked = std::panic::catch_unwind(AssertUnwindSafe(|| {
        (ctx.on_ready)(number_of_events);
    }))
    .is_err();

    if panicked {
        ctx.report_panic();
    }
}

/// A function that registers (or, with a null callback/user_data, clears) the
/// "on ready" push callback on an entity handle of type `H`. Implemented per
/// entity module so that `H`'s (private) lock and its specific
/// `rcl_*_set_on_new_*_callback` stay encapsulated there.
pub(crate) type SetOnReadyFn<H> = unsafe fn(&H, rcl_event_callback_t, *const c_void) -> rcl_ret_t;

/// Selects the registration guard for the specific rcl callback *slot* that a
/// [`SetOnReadyFn`] installs into.
///
/// rcl stores exactly one callback per entity slot. Registering a second
/// callback into a slot silently replaces the first, and later clearing the
/// first would then clear the *second* one, leaving that registration's
/// [`OnReadyHandle`] alive while its callback is no longer installed. rclrs
/// entities register a given slot exactly once and never replace it, so rather
/// than track generations to make replacement safe we simply reject a second,
/// overlapping registration on the same slot (see
/// [`RclrsError::OnReadyAlreadyRegistered`]).
///
/// Most primitives expose a single slot, but composite primitives (action
/// servers and clients) expose several distinct slots on one handle
/// (goal/cancel/result, feedback/status, ...). Pairing each registration with
/// the accessor for *its* slot lets independent slots on the same handle coexist
/// while still rejecting a true overlap on any one slot. The guard is an
/// `AtomicBool` owned by the entity handle and shared by every `Arc` clone of
/// it: `false` while the slot is free, `true` while a registration owns it.
pub(crate) type OnReadySlotFn<H> = fn(&H) -> &AtomicBool;

/// RAII registration of a push "on ready" callback on an rcl entity.
///
/// While alive, `on_ready(number_of_events)` is invoked by the middleware
/// whenever the entity becomes ready. Dropping it unregisters the callback
/// before releasing the context, so the middleware can never call into freed
/// memory.
pub(crate) struct OnReadyRegistration<H: Send + Sync + 'static> {
    set_callback: SetOnReadyFn<H>,
    // Selects the guard for the slot this registration owns, so `Drop` releases
    // the same slot it claimed in `new`.
    slot: OnReadySlotFn<H>,
    // Field order is important for teardown safety: `handle` is declared
    // (and therefore dropped) before `ctx`. Dropping the last `Arc<handle>`
    // finalizes the rcl entity (destroying the middleware reader), so by the
    // time `ctx` is freed no middleware thread can still invoke the trampoline
    // against it. This mirrors rclcpp, which frees its callback storage only
    // after `rcl_*_fini`. See `Drop` below.
    handle: Arc<H>,

    // Never read directly, held only so its `Drop` frees the callback context.
    #[allow(dead_code)]
    ctx: CtxBox,
}

impl<H: Send + Sync + 'static> OnReadyHandle for OnReadyRegistration<H> {}

impl<H: Send + Sync + 'static> OnReadyRegistration<H> {
    /// Register `on_ready` to be called by the middleware whenever the entity
    /// becomes ready. `set_callback` locks `handle` and installs the trampoline;
    /// `slot` selects the guard for the rcl callback slot `set_callback` targets.
    pub(crate) fn new(
        handle: Arc<H>,
        set_callback: SetOnReadyFn<H>,
        slot: OnReadySlotFn<H>,
        on_ready: Box<dyn Fn(usize) + Send + Sync>,
    ) -> Result<Self, RclrsError> {
        Self::new_with_panic_hook(handle, set_callback, slot, on_ready, None)
    }

    /// Like [`Self::new`], but invokes `on_panic` once if the user closure
    /// panics so an executor can surface the failure to `spin()`.
    pub(crate) fn new_with_panic_hook(
        handle: Arc<H>,
        set_callback: SetOnReadyFn<H>,
        slot: OnReadySlotFn<H>,
        on_ready: Box<dyn Fn(usize) + Send + Sync>,
        on_panic: Option<OnReadyPanicHook>,
    ) -> Result<Self, RclrsError> {
        // Claim this registration's callback slot. rcl stores one callback per
        // slot, so allowing an overlapping registration would let this one
        // silently clobber the existing callback (and dropping the earlier
        // registration would later clear ours). Reject the overlap instead of
        // corrupting the RAII semantics.
        let slot_taken = slot(&handle)
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_err();

        if slot_taken {
            return Err(RclrsError::OnReadyAlreadyRegistered);
        }

        let ctx = Box::into_raw(Box::new(EventCallbackCtx::new(on_ready, on_panic)));

        // SAFETY: `ctx` points to a live, heap-stable context that outlives the
        // registration (only freed once, when the `CtxBox` field is dropped).
        let result =
            unsafe { set_callback(&handle, Some(on_ready_trampoline), ctx as *const c_void).ok() };

        if let Err(err) = result {
            // Registration failed, so nothing else references `ctx`. Reclaim it.
            // SAFETY: `ctx` came from `Box::into_raw` above and was never
            // successfully registered.
            unsafe {
                drop(Box::from_raw(ctx));
            }
            // Release the slot we claimed so a later registration can succeed.
            slot(&handle).store(false, Ordering::Release);
            return Err(err);
        }

        Ok(Self {
            set_callback,
            slot,
            handle,
            ctx: CtxBox(ctx),
        })
    }
}

impl<H: Send + Sync + 'static> Drop for OnReadyRegistration<H> {
    fn drop(&mut self) {
        // Detach the callback so the middleware stops invoking the trampoline.
        // The context is NOT freed here, the `ctx: CtxBox` field is dropped
        // *after* `handle`, so the rcl entity is finalized before
        // the context is freed, avoiding a use-after-free if a callback is still
        // in flight at teardown.
        //
        // SAFETY: handle is valid and locked by the setter. A null callback +
        // null user_data clears the registration.
        unsafe {
            let _ = (self.set_callback)(&self.handle, None, std::ptr::null());
        }

        // Release this registration's callback slot so a new registration can be made.
        (self.slot)(&self.handle).store(false, Ordering::Release);
    }
}

/// Owns the heap-allocated [`EventCallbackCtx`] and frees it on drop. Kept as a
/// separate field of [`OnReadyRegistration`] so its drop runs *after* the
/// `handle` Arc.
struct CtxBox(*mut EventCallbackCtx);

// SAFETY: the pointer is only dereferenced by the middleware via the trampoline
// (forwarding to a `Send + Sync` closure). It carries no thread-unsafe state.
unsafe impl Send for CtxBox {}
unsafe impl Sync for CtxBox {}

impl Drop for CtxBox {
    fn drop(&mut self) {
        // SAFETY: by the time this runs, `OnReadyRegistration::drop` has cleared
        // the callback and the `handle` field has been dropped (finalizing the
        // rcl entity if it was the last reference), so no middleware thread can
        // still be dereferencing this context. Reclaim it exactly once.
        unsafe {
            drop(Box::from_raw(self.0));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        subscription::{set_subscription_on_new_message, subscription_on_ready_slot},
        *,
    };
    use ros_env::test_msgs::msg;
    use std::{
        sync::atomic::{AtomicBool, AtomicUsize, Ordering},
        time::{Duration, Instant},
    };

    /// The push callback fires when messages arrive, without ever spinning the
    /// executor (i.e. without `rcl_wait`).
    #[test]
    fn push_callback_fires_without_spinning() -> Result<(), RclrsError> {
        let executor = Context::default().create_basic_executor();
        let node = executor.create_node(&format!("test_push_callback_{}", line!()))?;
        let qos = QoSProfile::default().reliable().keep_last(10);

        let publisher = node.create_publisher::<msg::Empty>("test_push_topic".qos(qos))?;
        let subscription = node
            .create_subscription::<msg::Empty, _>("test_push_topic".qos(qos), |_: msg::Empty| {})?;

        let count = Arc::new(AtomicUsize::new(0));
        let count_cb = Arc::clone(&count);
        let _registration = OnReadyRegistration::new(
            Arc::clone(subscription.handle()),
            set_subscription_on_new_message,
            subscription_on_ready_slot,
            Box::new(move |n| {
                count_cb.fetch_add(n, Ordering::Relaxed);
            }),
        )?;

        // Publish repeatedly (to ride out discovery) and wait for the push
        // callback to fire. We deliberately never spin the executor.
        let deadline = Instant::now() + Duration::from_secs(10);
        while count.load(Ordering::Relaxed) == 0 && Instant::now() < deadline {
            publisher.publish(msg::Empty::default())?;
            std::thread::sleep(Duration::from_millis(20));
        }

        assert!(
            count.load(Ordering::Relaxed) > 0,
            "push callback never fired"
        );
        Ok(())
    }

    /// Rapidly create and drop registrations while messages are flowing. If the
    /// drop ordering is wrong (freeing the context before unregistering), the
    /// middleware thread can call into freed memory; this stresses that path.
    #[test]
    fn rapid_register_unregister_is_sound() -> Result<(), RclrsError> {
        let executor = Context::default().create_basic_executor();
        let node = executor.create_node(&format!("test_push_raii_{}", line!()))?;
        let qos = QoSProfile::default().reliable().keep_last(10);

        let publisher = node.create_publisher::<msg::Empty>("test_push_raii_topic".qos(qos))?;
        let subscription = node.create_subscription::<msg::Empty, _>(
            "test_push_raii_topic".qos(qos),
            |_: msg::Empty| {},
        )?;

        // A background thread floods the topic the whole time.
        let stop = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let stop_pub = Arc::clone(&stop);
        let flood = std::thread::spawn(move || {
            while !stop_pub.load(Ordering::Acquire) {
                let _ = publisher.publish(msg::Empty::default());
                std::thread::sleep(Duration::from_micros(50));
            }
        });

        // Register/unregister many times against the live subscription.
        for _ in 0..2000 {
            let count = Arc::new(AtomicUsize::new(0));
            let count_cb = Arc::clone(&count);
            let registration = OnReadyRegistration::new(
                Arc::clone(subscription.handle()),
                set_subscription_on_new_message,
                subscription_on_ready_slot,
                Box::new(move |n| {
                    count_cb.fetch_add(n, Ordering::Relaxed);
                }),
            )?;
            // Hold briefly so the middleware can fire into this context, then drop
            // (which must unregister before freeing).
            std::thread::sleep(Duration::from_micros(100));
            drop(registration);
        }

        stop.store(true, Ordering::Release);
        flood.join().unwrap();
        Ok(())
    }

    /// A panicking push callback must not unwind across the C trampoline or
    /// wedge later invocations. After the first panic the registration is
    /// disabled and the optional hook fires once.
    #[test]
    fn push_callback_panic_is_contained() -> Result<(), RclrsError> {
        let executor = Context::default().create_basic_executor();
        let node = executor.create_node(&format!("test_push_panic_{}", line!()))?;
        let qos = QoSProfile::default().reliable().keep_last(10);

        let publisher = node.create_publisher::<msg::Empty>("test_push_panic_topic".qos(qos))?;
        let subscription = node.create_subscription::<msg::Empty, _>(
            "test_push_panic_topic".qos(qos),
            |_: msg::Empty| {},
        )?;

        let invocations = Arc::new(AtomicUsize::new(0));
        let invocations_cb = Arc::clone(&invocations);
        let hook_fired = Arc::new(AtomicBool::new(false));
        let hook_fired_cb = Arc::clone(&hook_fired);
        let _registration = OnReadyRegistration::new_with_panic_hook(
            Arc::clone(subscription.handle()),
            set_subscription_on_new_message,
            subscription_on_ready_slot,
            Box::new(move |_| {
                invocations_cb.fetch_add(1, Ordering::Relaxed);
                panic!("push callback panic");
            }),
            Some(Arc::new(move || {
                hook_fired_cb.store(true, Ordering::Release);
            })),
        )?;

        let deadline = Instant::now() + Duration::from_secs(10);
        while invocations.load(Ordering::Relaxed) == 0 && Instant::now() < deadline {
            publisher.publish(msg::Empty::default())?;
            std::thread::sleep(Duration::from_millis(20));
        }

        assert_eq!(
            invocations.load(Ordering::Relaxed),
            1,
            "expected exactly one invocation before the registration was disabled",
        );
        assert!(
            hook_fired.load(Ordering::Acquire),
            "panic hook should fire once",
        );

        let invocations_after = invocations.load(Ordering::Relaxed);
        let deadline = Instant::now() + Duration::from_secs(2);
        while Instant::now() < deadline {
            publisher.publish(msg::Empty::default())?;
            std::thread::sleep(Duration::from_millis(20));
        }

        assert_eq!(
            invocations.load(Ordering::Relaxed),
            invocations_after,
            "registration should stay disabled after panic",
        );
        Ok(())
    }

    /// Only one push registration may own an entity's callback slot at a time.
    /// A second overlapping registration is rejected instead of silently
    /// clobbering the first, and once the first is dropped the slot is free for
    /// a fresh registration (which then fires normally). This guards against the
    /// RAII hazard where dropping the earlier handle would clear the later
    /// callback.
    #[test]
    fn overlapping_registration_is_rejected() -> Result<(), RclrsError> {
        let executor = Context::default().create_basic_executor();
        let node = executor.create_node("test_overlapping_registration_is_rejected")?;
        let qos = QoSProfile::default().reliable().keep_last(10);

        let publisher = node.create_publisher::<msg::Empty>("test_push_overlap_topic".qos(qos))?;
        let subscription = node.create_subscription::<msg::Empty, _>(
            "test_push_overlap_topic".qos(qos),
            |_: msg::Empty| {},
        )?;

        let first = OnReadyRegistration::new(
            Arc::clone(subscription.handle()),
            set_subscription_on_new_message,
            subscription_on_ready_slot,
            Box::new(|_| {}),
        )?;

        // A second registration against the same slot must be rejected while
        // the first is still alive.
        let second = OnReadyRegistration::new(
            Arc::clone(subscription.handle()),
            set_subscription_on_new_message,
            subscription_on_ready_slot,
            Box::new(|_| {}),
        );
        assert!(
            matches!(second, Err(RclrsError::OnReadyAlreadyRegistered)),
            "second overlapping registration should be rejected",
        );

        // Dropping the first releases the slot, so a new registration succeeds
        // and receives events.
        drop(first);
        let count = Arc::new(AtomicUsize::new(0));
        let count_cb = Arc::clone(&count);
        let _third = OnReadyRegistration::new(
            Arc::clone(subscription.handle()),
            set_subscription_on_new_message,
            subscription_on_ready_slot,
            Box::new(move |n| {
                count_cb.fetch_add(n, Ordering::Relaxed);
            }),
        )?;

        let deadline = Instant::now() + Duration::from_secs(10);
        while count.load(Ordering::Relaxed) == 0 && Instant::now() < deadline {
            publisher.publish(msg::Empty::default())?;
            std::thread::sleep(Duration::from_millis(20));
        }
        assert!(
            count.load(Ordering::Relaxed) > 0,
            "re-registration after drop never fired",
        );
        Ok(())
    }

    /// Models the composite-primitive (action server/client) case: several
    /// distinct rcl callback *slots* live on one handle. Each slot enforces its
    /// own single-registration invariant independently, so registrations on
    /// different slots of the same handle coexist, while a repeat registration
    /// on an already-owned slot is rejected. Uses a fake handle whose setters are
    /// no-ops so the test needs no middleware.
    #[test]
    fn independent_slots_on_one_handle_coexist() {
        struct MultiSlotHandle {
            slot_a: AtomicBool,
            slot_b: AtomicBool,
        }

        fn slot_a(handle: &MultiSlotHandle) -> &AtomicBool {
            &handle.slot_a
        }
        fn slot_b(handle: &MultiSlotHandle) -> &AtomicBool {
            &handle.slot_b
        }

        // The rcl setters are replaced with no-ops that report success, so the
        // registration logic runs without touching a real entity.
        unsafe fn set_noop(
            _handle: &MultiSlotHandle,
            _callback: rcl_event_callback_t,
            _user_data: *const c_void,
        ) -> rcl_ret_t {
            0 // RCL_RET_OK
        }

        let handle = Arc::new(MultiSlotHandle {
            slot_a: AtomicBool::new(false),
            slot_b: AtomicBool::new(false),
        });

        let reg_a =
            OnReadyRegistration::new(Arc::clone(&handle), set_noop, slot_a, Box::new(|_| {}))
                .expect("slot A should be free");
        // A different slot on the same handle is independent and must succeed.
        let reg_b =
            OnReadyRegistration::new(Arc::clone(&handle), set_noop, slot_b, Box::new(|_| {}))
                .expect("slot B is independent of slot A");

        // Re-registering either owned slot is rejected.
        assert!(
            matches!(
                OnReadyRegistration::new(Arc::clone(&handle), set_noop, slot_a, Box::new(|_| {})),
                Err(RclrsError::OnReadyAlreadyRegistered),
            ),
            "slot A is already owned",
        );
        assert!(
            matches!(
                OnReadyRegistration::new(Arc::clone(&handle), set_noop, slot_b, Box::new(|_| {})),
                Err(RclrsError::OnReadyAlreadyRegistered),
            ),
            "slot B is already owned",
        );

        // Dropping one registration frees only its own slot.
        drop(reg_a);
        assert!(
            !handle.slot_a.load(Ordering::Acquire),
            "slot A should be freed",
        );
        assert!(
            handle.slot_b.load(Ordering::Acquire),
            "slot B should still be owned by reg_b",
        );
        OnReadyRegistration::new(Arc::clone(&handle), set_noop, slot_a, Box::new(|_| {}))
            .expect("slot A should be reusable after its registration dropped");

        drop(reg_b);
    }
}

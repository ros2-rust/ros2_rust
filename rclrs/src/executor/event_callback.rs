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

use std::{os::raw::c_void, sync::Arc};

use crate::{rcl_bindings::*, OnReadyHandle, RclrsError, ToResult};

/// The context carried through rcl as `user_data`. Boxed so its address is
/// stable for the lifetime of the registration.
struct EventCallbackCtx {
    on_ready: Box<dyn Fn(usize) + Send + Sync>,
}

/// The C trampoline that rcl/rmw invokes when an entity becomes ready. It may be
/// called from a middleware thread, so it does nothing but forward to the Rust
/// closure. It must not run user code or take locks that could deadlock the
/// middleware.
unsafe extern "C" fn on_ready_trampoline(user_data: *const c_void, number_of_events: usize) {
    // SAFETY: `user_data` is the pointer to the `EventCallbackCtx` we passed to
    // the rcl setter. It stays valid until the owning registration's `Drop`
    // clears the callback, which always happens before the box is freed.
    let ctx = unsafe { &*(user_data as *const EventCallbackCtx) };
    (ctx.on_ready)(number_of_events);
}

/// A function that registers (or, with a null callback/user_data, clears) the
/// "on ready" push callback on an entity handle of type `H`. Implemented per
/// entity module so that `H`'s (private) lock and its specific
/// `rcl_*_set_on_new_*_callback` stay encapsulated there.
pub(crate) type SetOnReadyFn<H> = unsafe fn(&H, rcl_event_callback_t, *const c_void) -> rcl_ret_t;

/// RAII registration of a push "on ready" callback on an rcl entity.
///
/// While alive, `on_ready(number_of_events)` is invoked by the middleware
/// whenever the entity becomes ready. Dropping it unregisters the callback
/// before releasing the context, so the middleware can never call into freed
/// memory.
pub(crate) struct OnReadyRegistration<H: Send + Sync + 'static> {
    set_callback: SetOnReadyFn<H>,
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

/// Bundles several [`OnReadyHandle`]s into one, for composite primitives (action
/// servers/clients) that register a push callback per internal source. Dropping
/// it drops every contained registration, deregistering each callback.
// The Vec is never read — dropping it drops (and thus deregisters) every
// contained registration, which is the entire purpose of holding them.
pub(crate) struct CompositeOnReady(#[allow(dead_code)] pub(crate) Vec<Box<dyn OnReadyHandle>>);

impl OnReadyHandle for CompositeOnReady {}

impl<H: Send + Sync + 'static> OnReadyRegistration<H> {
    /// Register `on_ready` to be called by the middleware whenever the entity
    /// becomes ready. `set_callback` locks `handle` and installs the trampoline.
    pub(crate) fn new(
        handle: Arc<H>,
        set_callback: SetOnReadyFn<H>,
        on_ready: Box<dyn Fn(usize) + Send + Sync>,
    ) -> Result<Self, RclrsError> {
        let ctx = Box::into_raw(Box::new(EventCallbackCtx { on_ready }));

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
            return Err(err);
        }

        Ok(Self {
            set_callback,
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
    use crate::{subscription::set_subscription_on_new_message, *};
    use ros_env::test_msgs::msg;
    use std::{
        sync::atomic::{AtomicUsize, Ordering},
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
}

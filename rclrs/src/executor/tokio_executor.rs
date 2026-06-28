//! Event-driven, Tokio-backed executor for rclrs.
//!
//! Readiness is push-based (rcl `set_on_new_*_callback`), not polled. Each
//! **Worker** (the node's default group is its main worker) gets its own
//! `tokio::mpsc` mailbox and one spawned task that drains it. Because a Tokio
//! task is never polled by two threads at once, that single task gives
//! per-worker mutual exclusion *and* FIFO ordering for free; Tokio's scheduler
//! provides the thread pool, work-stealing, and M:N multiplexing. So different
//! workers run in parallel automatically, with no per-event spawn and nothing
//! for the user to configure.
//!
//! Worker tasks are **gated by spinning**: they only execute ROS entity
//! callbacks (subscriptions, services, clients, timers, actions) while `spin()`
//! is active, preserving rclrs's contract that those callbacks do not run until
//! you spin and that none are still running once `spin()` returns (quiescence is
//! enforced by waiting for in-flight callbacks before returning).
//!
//! This gating applies to entity callbacks, not to free-standing async tasks
//! spawned through the executor commands (e.g. `commands().run(..)`): those are
//! ordinary Tokio tasks and run on the runtime independently of `spin()`.
//! Code that needs work confined to spinning should put it in an entity callback
//! rather than a spawned task.

mod timer_scheduler;

use std::{
    any::Any,
    collections::HashMap,
    panic::AssertUnwindSafe,
    sync::{
        atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering},
        Arc, Mutex,
    },
    time::{Duration, Instant},
};

use futures::future::BoxFuture;
use tokio::sync::{
    mpsc::{UnboundedReceiver, UnboundedSender},
    watch, Notify,
};

use crate::{
    log_error, ActionClientReady, ActionServerReady, Context, ExecutorChannel, ExecutorRuntime,
    ExecutorWorkerOptions, OnReadyHandle, PayloadTask, RclPrimitiveKind, RclReturnCode, RclrsError,
    ReadyKind, SpinConditions, TimerSchedulerHandles, Waitable, WeakActivityListener,
    WorkerChannel,
};

use timer_scheduler::{TimerRegistration, TimerScheduler};

pub(crate) use timer_scheduler::TimerSchedulerNotify;

use super::Executor;

/// Identifies an entity within a worker.
type EntityId = u64;

/// A multi-threaded async executor backed by a Tokio runtime, driven by rcl push
/// callbacks, with one task per worker (see the module docs).
pub struct TokioExecutorRuntime {
    host: RuntimeHost,
    shared: Arc<ExecutorShared>,
}

impl TokioExecutorRuntime {
    /// Create an executor that owns a fresh multi-threaded Tokio runtime.
    ///
    /// Users should call [`CreateTokioExecutor::create_tokio_executor`] instead.
    pub(crate) fn new() -> Self {
        let runtime = tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()
            .expect("Failed to create Tokio runtime for rclrs executor");
        Self::with_runtime(runtime)
    }

    /// Create an executor that owns a caller-provided Tokio runtime.
    ///
    /// Users should call
    /// [`CreateTokioExecutor::create_tokio_executor_with_runtime`] instead.
    pub(crate) fn with_runtime(runtime: tokio::runtime::Runtime) -> Self {
        Self::from_host(RuntimeHost::Owned(runtime))
    }

    /// Create an executor that runs on a runtime adopted from the caller, rather
    /// than owning one.
    ///
    /// Users should call
    /// [`CreateTokioExecutor::create_tokio_executor_on_current_runtime`] or
    /// [`CreateTokioExecutor::create_tokio_executor_with_handle`] instead.
    pub(crate) fn with_handle(handle: tokio::runtime::Handle) -> Self {
        Self::from_host(RuntimeHost::Adopted(handle))
    }

    fn from_host(host: RuntimeHost) -> Self {
        let (spin, _) = watch::channel(false);
        let timer_scheduler = Arc::new(TimerScheduler::new());
        Self {
            host,
            shared: Arc::new(ExecutorShared {
                spin,
                halt: Arc::new(Notify::new()),
                active: Arc::new(AtomicUsize::new(0)),
                outstanding: Arc::new(AtomicUsize::new(0)),
                errors: Arc::new(Mutex::new(Vec::new())),
                next_entity_id: Arc::new(AtomicU64::new(0)),
                timer_scheduler,
            }),
        }
    }

    fn take_errors(&self) -> Vec<RclrsError> {
        std::mem::take(&mut *self.shared.errors.lock().unwrap())
    }

    /// If the spin is bounded by an until-promise, spawn a task that flips the
    /// halt flag and wakes the spin once the promise resolves, and return its
    /// handle. The caller aborts that handle when the spin ends, so a spin that
    /// stops for another reason (timeout, shutdown) does not leave the task
    /// parked on `promise.await` forever, accumulating one detached task per such
    /// spin.
    fn arm_until_promise(
        &self,
        conditions: &mut SpinConditions,
    ) -> Option<tokio::task::JoinHandle<()>> {
        let promise = conditions.options.until_promise_resolved.take()?;
        let halt_flag = Arc::clone(&conditions.halt_spinning);
        let halt_notify = Arc::clone(&self.shared.halt);
        Some(self.host.handle().spawn(async move {
            let _ = promise.await;
            halt_flag.store(true, Ordering::Release);
            halt_notify.notify_waiters();
        }))
    }

    /// The async core of spinning: open the gate so worker tasks run, wait until
    /// the spin should stop, close the gate, then wait for in-flight callbacks to
    /// finish (quiescence) before returning. Shared by the blocking `spin()` and
    /// the awaitable `spin_async()`. It never calls `block_on`, so it can run on
    /// any runtime, including the caller's in the adopted case.
    async fn run_spin(&self, mut conditions: SpinConditions) -> Vec<RclrsError> {
        let promise_task = self.arm_until_promise(&mut conditions);

        let stop_time = conditions.options.timeout.map(|t| Instant::now() + t);
        let only_once = conditions.options.only_next_available_work;

        // Open the gate so the worker tasks process, then wait until the spin
        // should stop.
        let _ = self.shared.spin.send(true);
        let timed_out = block_until_stop(
            Arc::clone(&conditions.halt_spinning),
            conditions.context.clone(),
            Arc::clone(&self.shared.active),
            Arc::clone(&self.shared.outstanding),
            Arc::clone(&self.shared.halt),
            stop_time,
            only_once,
        )
        .await;

        // Close the gate: worker tasks park after finishing any in-flight message.
        let _ = self.shared.spin.send(false);

        // Cancel the until-promise watcher so it doesn't outlive this spin.
        if let Some(task) = promise_task {
            task.abort();
        }

        // Wait for any in-flight callbacks to finish, so no ROS callback is
        // running once the spin returns (quiescence).
        await_quiescence(Arc::clone(&self.shared.active)).await;

        // Match the basic executor's contract: a timeout is reported as a
        // `Timeout` error rather than a silent return.
        let mut errors = self.take_errors();
        if timed_out {
            errors.push(RclrsError::RclError {
                code: RclReturnCode::Timeout,
                msg: None,
            });
        }
        errors
    }
}

impl ExecutorRuntime for TokioExecutorRuntime {
    fn channel(&self) -> Arc<dyn ExecutorChannel> {
        Arc::new(TokioExecutorChannel {
            handle: self.host.handle(),
            shared: Arc::clone(&self.shared),
        })
    }

    fn spin(&mut self, conditions: SpinConditions) -> Vec<RclrsError> {
        // `spin()` blocks the calling thread by driving the spin to completion. It
        // must not be called from within a Tokio runtime: it would block a runtime
        // worker thread (an immediate deadlock on a current-thread runtime), and
        // `block_on` from inside a runtime panics anyway. Give a clear message
        // instead, and steer callers in an async context to `spin_async().await`.
        assert!(
            tokio::runtime::Handle::try_current().is_err(),
            "Executor::spin() blocks the calling thread and must not be called from \
             within a Tokio runtime. Call Executor::spin_async().await instead.",
        );

        match &self.host {
            RuntimeHost::Owned(runtime) => runtime.block_on(self.run_spin(conditions)),
            // `Handle::block_on` is valid here because the assert above established
            // we are not inside a runtime. `clone()` avoids borrowing `self.host`
            // across the `self.run_spin(..)` borrow.
            RuntimeHost::Adopted(handle) => handle.clone().block_on(self.run_spin(conditions)),
        }
    }

    fn spin_async(
        self: Box<Self>,
        conditions: SpinConditions,
    ) -> BoxFuture<'static, (Box<dyn ExecutorRuntime>, Vec<RclrsError>)> {
        // Drive the spin on whatever runtime awaits this future (the caller's, in
        // the adopted case). The worker tasks run on the host runtime's threads
        // regardless, so no `block_on` and no helper thread are needed.
        Box::pin(async move {
            let errors = self.run_spin(conditions).await;
            (self as Box<dyn ExecutorRuntime>, errors)
        })
    }
}

/// Where the executor's tasks run: a runtime it owns, or one adopted from the
/// caller (e.g. the `#[tokio::main]` runtime).
enum RuntimeHost {
    /// A runtime this executor created. Used to `block_on` in the blocking
    /// `spin()`, and dropped (shutting down its worker threads) with the executor.
    Owned(tokio::runtime::Runtime),
    /// A runtime adopted from the caller. We only ever spawn on it; we never
    /// `block_on` it or shut it down.
    Adopted(tokio::runtime::Handle),
}

impl RuntimeHost {
    /// A handle for spawning, regardless of which kind of host this is.
    fn handle(&self) -> tokio::runtime::Handle {
        match self {
            RuntimeHost::Owned(runtime) => runtime.handle().clone(),
            RuntimeHost::Adopted(handle) => handle.clone(),
        }
    }
}

/// Block until the current spin should stop, returning whether it stopped because
/// the timeout elapsed (`true`) rather than because of a halt, a context
/// shutdown, or the available work draining (`false`).
///
/// With `only_once` (the spin_once pattern) it waits for work to arrive, drains
/// it, then returns; while work is in flight it never times out mid-batch. Without
/// it, it runs until `stop_time` (if any), a halt, or context shutdown.
async fn block_until_stop(
    halt: Arc<AtomicBool>,
    context: Context,
    active: Arc<AtomicUsize>,
    outstanding: Arc<AtomicUsize>,
    halt_notify: Arc<Notify>,
    stop_time: Option<Instant>,
    only_once: bool,
) -> bool {
    // For `only_once`, poll tightly so we detect the available work draining
    // without adding latency; otherwise a coarse poll is enough (we only re-check
    // halt/timeout/context).
    let poll = if only_once {
        Duration::from_micros(200)
    } else {
        Duration::from_millis(100)
    };

    // Whether any work has been seen this spin, so `only_once` waits for work to
    // arrive (up to the timeout) before declaring the batch drained.
    let mut saw_work = false;

    loop {
        if halt.load(Ordering::Acquire) {
            return false;
        }

        // Stop spinning once the ROS context is no longer valid (shutdown).
        if !context.ok() {
            return false;
        }

        let busy = outstanding.load(Ordering::Acquire) > 0 || active.load(Ordering::Acquire) > 0;
        if busy {
            saw_work = true;
        }

        if only_once {
            // Process the currently-available work, then stop. While work is in
            // flight we keep draining (never time out mid-batch); once it has
            // drained we're done. If no work is in flight we wait for some to
            // arrive, up to the timeout.
            if saw_work && !busy {
                return false;
            }

            if !busy && stop_time.is_some_and(|st| Instant::now() >= st) {
                return true;
            }
        } else if stop_time.is_some_and(|st| Instant::now() >= st) {
            // Ran for the requested duration.
            return true;
        }

        let wait = stop_time
            .map(|st| st.saturating_duration_since(Instant::now()))
            .unwrap_or(poll)
            .min(poll);

        tokio::select! {
            _ = halt_notify.notified() => {}
            _ = tokio::time::sleep(wait) => {}
        }
    }
}

/// Wait until no callbacks are in flight across any worker. Used by `spin()` to
/// uphold quiescence before returning.
async fn await_quiescence(active: Arc<AtomicUsize>) {
    while active.load(Ordering::Acquire) > 0 {
        tokio::time::sleep(Duration::from_micros(100)).await;
    }
}

/// This trait allows [`Context`] to create a Tokio-based executor.
///
/// There are two families of constructors. The `create_tokio_executor*` methods
/// give the executor its **own** runtime. The `*_on_current_runtime` /
/// `*_with_handle` methods **adopt** a runtime the caller already has (for
/// example the one set up by `#[tokio::main]`), so ROS callbacks and the caller's
/// other Tokio work share a single runtime and thread pool.
///
/// Inside `#[tokio::main]`, prefer the adopting constructors. An owned-runtime
/// executor is meant for a non-async context: its blocking `spin` cannot run
/// inside a runtime, and dropping the owned runtime from within a runtime panics
/// (a Tokio rule). The adopting constructors avoid both, since they own no
/// runtime.
///
/// When adopting a runtime, note that:
///
/// - The blocking [`Executor::spin`] must not be called from within a runtime;
///   use [`Executor::spin_async`]`.await` from async code. (This is true even
///   for an owned runtime: `spin` blocks the calling thread.)
/// - The adopted runtime must have Tokio's time driver enabled (`enable_time` or
///   `enable_all`; `#[tokio::main]` does by default), since timers and the
///   per-worker reap interval rely on it.
/// - A current-thread runtime is fine for async or short non-blocking callbacks,
///   but a blocking or CPU-bound sync callback occupies the single thread, and
///   `tokio::task::block_in_place` (the usual escape hatch) panics on a
///   current-thread runtime. Use a multi-threaded runtime if callbacks can block.
pub trait CreateTokioExecutor {
    /// Create an event-driven Tokio-based executor associated with this
    /// [`Context`], with its own default multi-threaded Tokio runtime.
    fn create_tokio_executor(&self) -> Executor;

    /// Create an event-driven Tokio-based executor with a caller-provided Tokio
    /// runtime (e.g. to control worker-thread count or names). The executor owns
    /// the runtime.
    fn create_tokio_executor_with_runtime(&self, runtime: tokio::runtime::Runtime) -> Executor;

    /// Create an event-driven Tokio-based executor that runs on the **current**
    /// Tokio runtime instead of creating its own. Must be called from within a
    /// runtime (panics otherwise, like [`tokio::runtime::Handle::current`]). See
    /// the trait docs for the time-driver and current-thread caveats.
    fn create_tokio_executor_on_current_runtime(&self) -> Executor;

    /// Create an event-driven Tokio-based executor that runs on the runtime
    /// behind `handle`, instead of creating its own. Like
    /// [`create_tokio_executor_on_current_runtime`][Self::create_tokio_executor_on_current_runtime]
    /// but for callers that hold a [`Handle`][tokio::runtime::Handle] without
    /// being on a runtime thread at construction time.
    fn create_tokio_executor_with_handle(&self, handle: tokio::runtime::Handle) -> Executor;
}

impl CreateTokioExecutor for Context {
    fn create_tokio_executor(&self) -> Executor {
        self.create_executor(TokioExecutorRuntime::new())
    }

    fn create_tokio_executor_with_runtime(&self, runtime: tokio::runtime::Runtime) -> Executor {
        self.create_executor(TokioExecutorRuntime::with_runtime(runtime))
    }

    fn create_tokio_executor_on_current_runtime(&self) -> Executor {
        self.create_executor(TokioExecutorRuntime::with_handle(
            tokio::runtime::Handle::current(),
        ))
    }

    fn create_tokio_executor_with_handle(&self, handle: tokio::runtime::Handle) -> Executor {
        self.create_executor(TokioExecutorRuntime::with_handle(handle))
    }
}

struct TokioExecutorChannel {
    handle: tokio::runtime::Handle,
    shared: Arc<ExecutorShared>,
}

impl ExecutorChannel for TokioExecutorChannel {
    fn create_worker(&self, options: ExecutorWorkerOptions) -> Arc<dyn WorkerChannel> {
        let (mailbox_tx, mailbox_rx) = tokio::sync::mpsc::unbounded_channel();
        let entities = Arc::new(Mutex::new(HashMap::new()));
        let listeners = Arc::new(Mutex::new(Vec::new()));

        // One task per worker. Tokio schedules it. Different workers therefore run
        // concurrently, while this worker's callbacks stay serialized and ordered.
        // The reap `Interval` is created inside the spawned task (i.e. within the
        // Tokio runtime), which is where a Tokio timer must be constructed.
        let worker_entities = Arc::clone(&entities);
        let worker_listeners = Arc::clone(&listeners);
        let spinning = self.shared.spin.subscribe();
        let error_sink = Arc::clone(&self.shared.errors);
        let active = Arc::clone(&self.shared.active);
        let outstanding = Arc::clone(&self.shared.outstanding);
        let payload = options.payload;
        self.handle.spawn(async move {
            WorkerLoop {
                mailbox: mailbox_rx,
                entities: worker_entities,
                payload,
                listeners: worker_listeners,
                spinning,
                error_sink,
                active,
                outstanding,
                reap: tokio::time::interval(Duration::from_secs(1)),
            }
            .run()
            .await
        });

        Arc::new(TokioWorkerChannel {
            handle: self.handle.clone(),
            mailbox: mailbox_tx,
            entities,
            listeners,
            errors: Arc::clone(&self.shared.errors),
            next_entity_id: Arc::clone(&self.shared.next_entity_id),
            outstanding: Arc::clone(&self.shared.outstanding),
            timer_scheduler: Arc::clone(&self.shared.timer_scheduler),
        })
    }

    fn wake_all_wait_sets(&self) {
        // Wake any in-progress spin so it re-checks halt_spinning promptly.
        self.shared.halt.notify_waiters();
    }
}

struct TokioWorkerChannel {
    handle: tokio::runtime::Handle,
    mailbox: UnboundedSender<WorkerMsg>,
    entities: Arc<Mutex<HashMap<EntityId, Arc<WorkerEntity>>>>,
    listeners: Arc<Mutex<Vec<WeakActivityListener>>>,
    errors: Arc<Mutex<Vec<RclrsError>>>,
    next_entity_id: Arc<AtomicU64>,
    outstanding: Arc<AtomicUsize>,
    timer_scheduler: Arc<TimerScheduler>,
}

impl TokioWorkerChannel {
    /// Build the push-callback closure for an entity. It forwards each
    /// middleware notification (with its event count) to the entity's
    /// [`EntityDispatch`], which combines repeated notifications so the mailbox
    /// holds at most one pending `Ready` per entity even while spinning is paused.
    fn make_on_ready(
        &self,
        dispatch: Arc<EntityDispatch>,
        ready: Option<Arc<Mutex<ReadyKind>>>,
    ) -> Box<dyn Fn(ReadyKind, usize) + Send + Sync> {
        Box::new(move |kind, count| {
            // Composite primitives merge their per-sub-entity readiness; simple
            // ones have no accumulator and stay lock-free.
            if let Some(acc) = &ready {
                merge_ready(&mut acc.lock().unwrap(), kind);
            }
            dispatch.notify(count.max(1));
        })
    }
}

impl WorkerChannel for TokioWorkerChannel {
    fn add_async_task(&self, f: BoxFuture<'static, ()>) {
        self.handle.spawn(f);
    }

    /// Register a worker entity with the executor: insert it into the registry,
    /// then install its push callback (for message-driven primitives) or register
    /// it with the shared timer scheduler (for timers), so the entity's readiness
    /// reaches this worker's mailbox.
    ///
    /// Guard conditions have no rcl push-callback API, so `register_on_ready`
    /// returns `None` and they sit inert here. That is correct for the per-worker
    /// wakeup guard conditions: there is no `rcl_wait` to interrupt, since new
    /// entities register their callback immediately, payload tasks go straight to
    /// the mailbox, and removals are reaped. The one guard condition that does
    /// carry a callback is the node graph guard condition (see `node_options.rs`);
    /// rmw exposes no "on trigger" callback for guard conditions, so on this
    /// executor graph changes are not event-driven. This is not a correctness
    /// regression: graph-change listeners (`Node::notify_on_graph_change`)
    /// re-check their condition on a period regardless of notifications, so they
    /// still resolve, just within that period rather than immediately.
    /// Lower-latency graph-change handling is left as future work.
    fn add_to_wait_set(&self, new_entity: Waitable) {
        let id = self.next_entity_id.fetch_add(1, Ordering::Relaxed);
        let kind = new_entity.kind();

        // Composite primitives (action servers/clients) report different readiness
        // per sub-entity through the same entity; accumulate the merged readiness
        // here. Simple primitives are always `Basic` and skip this (lock-free).
        let ready: Option<Arc<Mutex<ReadyKind>>> = match kind {
            RclPrimitiveKind::ActionServer => Some(Arc::new(Mutex::new(ReadyKind::ActionServer(
                ActionServerReady::default(),
            )))),
            RclPrimitiveKind::ActionClient => Some(Arc::new(Mutex::new(ReadyKind::ActionClient(
                ActionClientReady::default(),
            )))),
            _ => None,
        };

        // Shared readiness dispatch: combines repeated notifications into at most
        // one pending `Ready` per entity, with the event count accumulated (and,
        // for composite primitives, readiness flags merged in `ready`).
        let dispatch = Arc::new(EntityDispatch::new(
            id,
            self.mailbox.clone(),
            Arc::clone(&self.outstanding),
        ));
        let on_ready = self.make_on_ready(Arc::clone(&dispatch), ready.clone());

        // Grab the timer-scheduler inputs before `new_entity` is moved into the
        // registry below.
        let timer_handles = new_entity.timer_scheduler_handles();
        let in_use = new_entity.in_use_handle();

        // Insert into the registry BEFORE registering the push callback (or the
        // timer), so the entity is always resolvable by the time any readiness
        // can enqueue a `Ready` for it.
        //
        // Registering first would race: an early middleware callback could fire,
        // enqueue a `Ready`, and have the worker drop it (entity not found yet),
        // leaving the dispatch flag stuck set so no further `Ready` is ever sent.
        // The `_on_ready` handle is filled in just below, once the callback is live.
        let entry = Arc::new(WorkerEntity {
            waitable: Mutex::new(new_entity),
            dispatch: Arc::clone(&dispatch),
            ready: ready.clone(),
            _on_ready: Mutex::new(None),
        });
        self.entities.lock().unwrap().insert(id, Arc::clone(&entry));

        // Now register the push callback against the (already-inserted) entity.
        // Holding the waitable lock here is safe: the callback only touches the
        // entity's `dispatch` (atomics + mailbox), never the waitable.
        let registration = match entry.waitable.lock().unwrap().register_on_ready(on_ready) {
            Ok(registration) => registration,
            Err(err) => {
                // Surface the failure both in the log and via spin()'s error
                // return, rather than silently leaving an inert entity.
                log_error!(
                    "rclrs.executor.tokio_executor",
                    "Failed to register an on-ready callback: {err}",
                );
                self.errors.lock().unwrap().push(err);
                None
            }
        };
        *entry._on_ready.lock().unwrap() = registration;

        // Timers have no rcl push callback. Register them with the shared timer
        // scheduler, which paces all timers from one thread and delivers each fire
        // through the same `dispatch` as push callbacks, so a tick is one bounded
        // take like any other event.
        if let Some(TimerSchedulerHandles {
            rcl_timer,
            notify_slot,
        }) = timer_handles
        {
            let notify = self.timer_scheduler.register(TimerRegistration {
                rcl_timer,
                dispatch: Arc::clone(&dispatch),
                in_use: Arc::clone(&in_use),
            });
            *notify_slot.lock().unwrap() = Some(notify);
        }

        // Action-server goal expiration has no rcl push callback (rcl uses an
        // internal timer); poll it periodically so completed goals are cleaned up.
        if kind == RclPrimitiveKind::ActionServer {
            if let Some(acc) = ready {
                self.handle
                    .spawn(action_expire_driver(in_use, Arc::clone(&dispatch), acc));
            }
        }
    }

    fn send_payload_task(&self, f: PayloadTask) {
        // Counts as outstanding work until a worker handles it (so
        // `only_next_available_work` waits for payload tasks too).
        self.outstanding.fetch_add(1, Ordering::AcqRel);
        let _ = self.mailbox.send(WorkerMsg::Payload(f));
    }

    fn add_activity_listener(&self, listener: WeakActivityListener) {
        self.listeners.lock().unwrap().push(listener);
    }
}

/// State shared between the runtime and all workers.
struct ExecutorShared {
    /// Gate the worker tasks observe: they execute only while this is `true`.
    spin: watch::Sender<bool>,

    /// Promptly wakes `spin()` when a halt is requested.
    halt: Arc<Notify>,

    /// Number of callbacks currently executing across all workers. `spin()`
    /// waits for this to reach zero before returning, so no ROS callback is
    /// running once `spin()` has returned (quiescence).
    active: Arc<AtomicUsize>,

    /// Number of mailbox messages enqueued across all workers but not yet
    /// handled (queued *or* in flight). `spin()` with `only_next_available_work`
    /// uses this to detect when the currently-available work has drained.
    outstanding: Arc<AtomicUsize>,

    /// Errors produced by callbacks; drained and returned by `spin()`.
    errors: Arc<Mutex<Vec<RclrsError>>>,

    /// Allocates entity ids across all workers.
    next_entity_id: Arc<AtomicU64>,

    /// Shared timer scheduler (one heap, one waiter) for all workers' timers.
    timer_scheduler: Arc<TimerScheduler>,
}

/// The per-worker event loop. Spawned once per worker (see
/// [`TokioExecutorChannel::create_worker`]). It owns the worker's payload and
/// drains its mailbox for the executor's lifetime, running each message against
/// the payload (and the worker's activity listeners) while the executor is
/// spinning.
struct WorkerLoop {
    mailbox: UnboundedReceiver<WorkerMsg>,
    entities: Arc<Mutex<HashMap<EntityId, Arc<WorkerEntity>>>>,
    payload: Box<dyn Any + Send>,
    listeners: Arc<Mutex<Vec<WeakActivityListener>>>,
    spinning: watch::Receiver<bool>,
    error_sink: Arc<Mutex<Vec<RclrsError>>>,

    /// Callbacks currently in flight across all workers; `spin()` waits for this
    /// to reach zero before returning (quiescence).
    active: Arc<AtomicUsize>,

    /// Mailbox messages enqueued but not yet handled; `spin()` uses this to know
    /// when the currently-available work has drained.
    outstanding: Arc<AtomicUsize>,

    /// Fires periodically so the loop can drop entities whose owning handle has
    /// been released (see [`next_message`][Self::next_message]). Must be created
    /// inside the Tokio runtime, so the loop is constructed in the spawned task.
    reap: tokio::time::Interval,
}

impl WorkerLoop {
    /// Drain the mailbox for the executor's lifetime. Each message is gated on
    /// the executor spinning, then handled. Returns when the worker's mailbox is
    /// dropped or the executor itself is dropped.
    async fn run(mut self) {
        loop {
            let Some(msg) = self.next_message().await else {
                return; // worker dropped
            };

            if !self.wait_until_spinning().await {
                return; // executor dropped
            }

            self.handle(msg);

            // The message is fully handled: it is no longer in flight, and no
            // longer counts as outstanding work for `spin()`.
            self.active.fetch_sub(1, Ordering::AcqRel);
            self.outstanding.fetch_sub(1, Ordering::AcqRel);
        }
    }

    /// Wait for the next mailbox message, reaping dropped entities on the side.
    ///
    /// On a periodic tick we drop entities whose owning handle has been released,
    /// so we stop holding their rcl handle and push-callback registration.
    /// Entities on active topics are also reaped on-event in
    /// [`run_ready_entity`][Self::run_ready_entity]; this catches idle ones.
    /// Returns `None` once the worker (its mailbox sender) has been dropped.
    async fn next_message(&mut self) -> Option<WorkerMsg> {
        // Split the borrow so `select!` can poll the reap timer and the mailbox
        // (two separate fields) at the same time.
        let Self {
            reap,
            mailbox,
            entities,
            ..
        } = self;
        loop {
            tokio::select! {
                _ = reap.tick() => {
                    entities
                        .lock()
                        .unwrap()
                        .retain(|_, e| e.waitable.lock().unwrap().in_use());
                }
                msg = mailbox.recv() => return msg,
            }
        }
    }

    /// Count the pending message as in-flight and block until the executor is
    /// spinning.
    ///
    /// The in-flight count is incremented *before* the gate is checked, so a
    /// concurrent `spin()` closing the gate either observes this unit (and waits
    /// for it) or has already closed the gate before we run anything. Either way
    /// no callback runs after `spin()` returns. While parked the count is released
    /// so it does not hold up quiescence.
    ///
    /// Returns `true` once spinning, with the in-flight count left incremented;
    /// the caller decrements it once the message is handled. Returns `false` if
    /// the executor was dropped, with the in-flight count already released.
    async fn wait_until_spinning(&mut self) -> bool {
        self.active.fetch_add(1, Ordering::AcqRel);

        loop {
            if *self.spinning.borrow_and_update() {
                return true;
            }

            self.active.fetch_sub(1, Ordering::AcqRel);

            if self.spinning.changed().await.is_err() {
                return false; // executor dropped
            }

            self.active.fetch_add(1, Ordering::AcqRel);
        }
    }

    /// Run a single mailbox message against the worker's payload.
    fn handle(&mut self, msg: WorkerMsg) {
        match msg {
            WorkerMsg::Ready { entity } => {
                let errors = self.run_ready_entity(entity);
                if !errors.is_empty() {
                    self.error_sink.lock().unwrap().extend(errors);
                }
            }
            WorkerMsg::Payload(task) => {
                // Contain a panic so a bad task cannot kill the worker.
                if std::panic::catch_unwind(AssertUnwindSafe(|| task(&mut *self.payload))).is_err()
                {
                    log_error!(
                        "rclrs.executor.tokio_executor",
                        "A payload task panicked; the executor contained the panic \
                         and continues.",
                    );
                }
            }
        }
    }

    /// Handle a `Ready` notification for `entity`: re-arm notification combining,
    /// then either
    /// run its callback(s) or, if its owning handle has been dropped, deregister
    /// it. Returns any errors the callbacks produced.
    fn run_ready_entity(&mut self, entity: EntityId) -> Vec<RclrsError> {
        // Clone the entry out under a brief lock so a callback may create new
        // entities on this worker without deadlocking.
        let Some(entry) = self.entities.lock().unwrap().get(&entity).cloned() else {
            return Vec::new();
        };

        // Take all pending events and clear the queued flag; a notification that
        // races this re-arms and queues a fresh `Ready`, so no wakeup is lost
        // (see [`EntityDispatch::take_pending`]).
        let count = entry.dispatch.take_pending();
        let ready = match &entry.ready {
            None => ReadyKind::Basic,
            Some(acc) => {
                let mut acc = acc.lock().unwrap();
                let taken = *acc;
                *acc = neutral_ready(&taken);
                taken
            }
        };

        let mut waitable = entry.waitable.lock().unwrap();
        if !waitable.in_use() {
            // The owning handle was dropped: deregister and never run a callback
            // for a dropped entity.
            drop(waitable);
            self.entities.lock().unwrap().remove(&entity);
            return Vec::new();
        }

        let (ran, errors) = Self::execute_ready(&mut waitable, ready, count, &mut *self.payload);
        drop(waitable);

        if ran {
            self.run_listeners_contained();
        }
        errors
    }

    /// Take up to `count` items from `waitable` and run its callback for each,
    /// stopping early once a take turns up empty.
    ///
    /// The work runs inside `catch_unwind` so a panicking callback cannot kill
    /// the worker task or leak the `active`/`outstanding` counters that `spin()`
    /// waits on (which would wedge quiescence forever). The mutex guard is held
    /// by the caller *outside* the closure, so it drops normally (unpoisoned) if
    /// the callback unwinds. Returns whether any callback ran and any errors.
    fn execute_ready(
        waitable: &mut Waitable,
        ready: ReadyKind,
        count: usize,
        payload: &mut dyn Any,
    ) -> (bool, Vec<RclrsError>) {
        let exec = std::panic::catch_unwind(AssertUnwindSafe(|| {
            let mut ran = false;
            let mut errors = Vec::new();
            for _ in 0..count.max(1) {
                // SAFETY: `payload` is this worker's payload and `waitable` was
                // registered on this worker, so its primitive expects exactly
                // this payload type.
                match unsafe { waitable.execute_with(ready, payload) } {
                    Ok(()) => ran = true,
                    Err(err) if err.is_take_failed() => break,
                    Err(err) => {
                        errors.push(err);
                        break;
                    }
                }
            }
            (ran, errors)
        }));

        exec.unwrap_or_else(|_| {
            log_error!(
                "rclrs.executor.tokio_executor",
                "A callback panicked while spinning; the executor contained the \
                 panic and continues. The worker's payload may now be in an \
                 inconsistent state.",
            );
            (false, Vec::new())
        })
    }

    /// Run this worker's activity listeners against the payload, containing any
    /// panic so a bad listener cannot kill the worker task.
    fn run_listeners_contained(&mut self) {
        if std::panic::catch_unwind(AssertUnwindSafe(|| {
            crate::worker::run_activity_listeners(&self.listeners, &mut *self.payload);
        }))
        .is_err()
        {
            log_error!(
                "rclrs.executor.tokio_executor",
                "A worker activity listener panicked; the executor contained the \
                 panic and continues.",
            );
        }
    }
}

/// A message delivered to a worker's task.
enum WorkerMsg {
    /// The entity became ready, take and run its callback(s). At most one such
    /// message is outstanding per entity at a time, and the worker takes all
    /// pending events when it handles it (see [`EntityDispatch`]).
    Ready { entity: EntityId },

    /// Run a one-shot task against the worker's payload.
    Payload(PayloadTask),
}

/// Delivers an entity's readiness to its worker, combining repeated
/// notifications so the mailbox holds at most one pending `Ready` per entity.
///
/// The middleware can fire many notifications for one entity in a row. Rather
/// than queue one mailbox message per notification, we combine them: `scheduled`
/// is set while a `Ready` for this entity is already queued and not yet handled,
/// so further notifications only add their event count to `pending` instead of
/// queueing another message. That keeps the mailbox to at most one pending
/// `Ready` per entity even while spinning is paused, and no work is lost: the
/// worker takes exactly `pending` items. `outstanding` is the executor-wide
/// in-flight counter `spin()` uses for quiescence.
///
/// Shared (behind an `Arc`) by every path that can make an entity ready — the
/// push callback, the timer scheduler, and the worker that drains it.
pub(crate) struct EntityDispatch {
    /// Identifies the entity in `Ready` messages.
    id: EntityId,
    /// The owning worker's mailbox.
    mailbox: UnboundedSender<WorkerMsg>,
    /// Set while a `Ready` for this entity is queued and not yet handled.
    scheduled: AtomicBool,
    /// Ready events reported but not yet taken by the worker.
    pending: AtomicUsize,
    /// Executor-wide count of queued-but-unhandled messages, for `spin()`.
    outstanding: Arc<AtomicUsize>,
}

impl EntityDispatch {
    fn new(
        id: EntityId,
        mailbox: UnboundedSender<WorkerMsg>,
        outstanding: Arc<AtomicUsize>,
    ) -> Self {
        Self {
            id,
            mailbox,
            scheduled: AtomicBool::new(false),
            pending: AtomicUsize::new(0),
            outstanding,
        }
    }

    /// Record `events` ready events and queue a `Ready` if one isn't already in
    /// flight. Returns `false` if the worker's mailbox is closed.
    pub(crate) fn notify(&self, events: usize) -> bool {
        self.pending.fetch_add(events, Ordering::AcqRel);
        if self.scheduled.swap(true, Ordering::AcqRel) {
            true
        } else {
            self.enqueue_ready()
        }
    }

    /// Like [`notify`][Self::notify] but counts at most one pending event: for
    /// idempotent readiness (e.g. action-goal expiry) where a single run clears
    /// everything, so a queued-but-unhandled `Ready` must not accumulate more.
    pub(crate) fn notify_once(&self) -> bool {
        if self.scheduled.swap(true, Ordering::AcqRel) {
            true
        } else {
            self.pending.fetch_add(1, Ordering::AcqRel);
            self.enqueue_ready()
        }
    }

    /// Whether a `Ready` is currently queued and not yet handled.
    pub(crate) fn is_scheduled(&self) -> bool {
        self.scheduled.load(Ordering::Acquire)
    }

    /// Worker side: clear the queued flag and take all pending events. Clearing
    /// *before* the take lets a notification that races the drain queue a fresh
    /// `Ready`, so no wakeup is lost.
    fn take_pending(&self) -> usize {
        self.scheduled.store(false, Ordering::Release);
        self.pending.swap(0, Ordering::AcqRel)
    }

    /// Count one outstanding message and send the `Ready`. Returns `false` if the
    /// mailbox is closed.
    fn enqueue_ready(&self) -> bool {
        self.outstanding.fetch_add(1, Ordering::AcqRel);
        self.mailbox
            .send(WorkerMsg::Ready { entity: self.id })
            .is_ok()
    }
}

/// An entity owned by a worker: registration inserts it, the worker task runs it.
struct WorkerEntity {
    waitable: Mutex<Waitable>,

    /// Delivers this entity's readiness to the worker, combining repeated
    /// notifications into at most one pending `Ready` (see [`EntityDispatch`]).
    /// Shared with the entity's push callback and, for timers, the scheduler.
    dispatch: Arc<EntityDispatch>,

    /// Merged readiness for composite primitives (action servers/clients) whose
    /// sub-entities report *different* [`ReadyKind`]s through the same entity.
    /// Notifications OR their flags in; the worker swaps it out (resetting to the
    /// kind's neutral value) and runs the primitive with it. `None` for primitives
    /// with a single readiness path (subscriptions/services/clients/timers), which
    /// are always [`ReadyKind::Basic`] — keeping their hot path lock-free.
    ready: Option<Arc<Mutex<ReadyKind>>>,

    /// Keeps the push callback registered; dropping it deregisters. `None` for
    /// passive entities (e.g. guard conditions) or timers (driven separately).
    /// Behind a `Mutex` so it can be filled in *after* the entity is inserted
    /// into the registry: registering before insertion would let an early
    /// middleware callback enqueue a `Ready` the worker can't resolve, which it
    /// would drop, wedging the entity with its dispatch flag stuck set.
    _on_ready: Mutex<Option<Box<dyn OnReadyHandle>>>,
}

/// OR the readiness flags of `new` into `acc`. Used to merge the
/// per-sub-entity readiness of an action server/client into one value before the
/// worker runs the primitive. Basic and mismatched variants leave `acc` as-is.
fn merge_ready(acc: &mut ReadyKind, new: ReadyKind) {
    match (acc, new) {
        (ReadyKind::ActionServer(a), ReadyKind::ActionServer(b)) => {
            a.goal_request |= b.goal_request;
            a.cancel_request |= b.cancel_request;
            a.result_request |= b.result_request;
            a.goal_expired |= b.goal_expired;
        }
        (ReadyKind::ActionClient(a), ReadyKind::ActionClient(b)) => {
            a.feedback |= b.feedback;
            a.status |= b.status;
            a.goal_response |= b.goal_response;
            a.cancel_response |= b.cancel_response;
            a.result_response |= b.result_response;
        }
        _ => {}
    }
}

/// The "no readiness" value for `kind`'s variant, used to reset an accumulator
/// after the worker has taken its merged readiness.
fn neutral_ready(kind: &ReadyKind) -> ReadyKind {
    match kind {
        ReadyKind::Basic => ReadyKind::Basic,
        ReadyKind::ActionServer(_) => ReadyKind::ActionServer(ActionServerReady::default()),
        ReadyKind::ActionClient(_) => ReadyKind::ActionClient(ActionClientReady::default()),
    }
}

/// How often to poll an action server for expired goals. Goal expiration is
/// driven by an rcl-internal timer with no push callback, so we poll instead.
/// The interval only bounds how promptly a *completed* goal is cleaned up
/// (typically well after its multi-second result timeout), so it can be coarse.
const ACTION_EXPIRE_POLL: Duration = Duration::from_millis(100);

/// Periodically nudge an action server to expire completed goals (there is no
/// rcl push callback for expiration). Enqueues a `goal_expired` readiness through
/// the same coalescing path as other events; the worker runs
/// `rcl_action_expire_goals`, which is a cheap no-op when nothing has expired.
/// Stops once the action server's owning entity is dropped.
async fn action_expire_driver(
    in_use: Arc<AtomicBool>,
    dispatch: Arc<EntityDispatch>,
    ready: Arc<Mutex<ReadyKind>>,
) {
    loop {
        tokio::time::sleep(ACTION_EXPIRE_POLL).await;
        if !in_use.load(Ordering::Acquire) {
            return;
        }
        // Flag expiry as ready and deliver it through the dispatch. Expiry is
        // idempotent (one run clears all expired goals), so `notify_once` keeps
        // at most one pending take even while spinning is paused — otherwise the
        // 100ms poll would accumulate redundant work for the whole pause.
        merge_ready(
            &mut ready.lock().unwrap(),
            ReadyKind::ActionServer(ActionServerReady {
                goal_expired: true,
                ..Default::default()
            }),
        );
        if !dispatch.notify_once() {
            return; // worker gone
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::*;
    use ros_env::{test_msgs, test_msgs::msg};
    use std::{
        sync::{
            atomic::{AtomicBool, AtomicUsize, Ordering},
            Arc,
        },
        time::{Duration, Instant},
    };

    /// The executor can adopt the current Tokio runtime (the `#[tokio::main]`
    /// pattern) instead of owning one, and deliver messages while driven by
    /// `spin_async().await` from within that runtime.
    #[tokio::test(flavor = "multi_thread")]
    async fn tokio_adopts_current_runtime() -> Result<(), RclrsError> {
        let mut executor = Context::default().create_tokio_executor_on_current_runtime();
        let node = executor.create_node(
            format!("test_tokio_adopt_mt_{}", line!()).start_parameter_services(false),
        )?;
        let qos = QoSProfile::default().reliable().keep_last(10);

        let received = Arc::new(AtomicUsize::new(0));
        let received_cb = Arc::clone(&received);
        let _sub = node.create_subscription::<msg::Empty, _>(
            "tokio_adopt_mt_topic".qos(qos),
            move |_m: msg::Empty| {
                received_cb.fetch_add(1, Ordering::Relaxed);
            },
        )?;
        let publisher = node.create_publisher::<msg::Empty>("tokio_adopt_mt_topic".qos(qos))?;

        // Drive the executor with `spin_async` on the current runtime (no separate
        // runtime, no helper thread), republishing to ride out discovery until a
        // message arrives.
        let deadline = Instant::now() + Duration::from_secs(10);
        while received.load(Ordering::Relaxed) == 0 && Instant::now() < deadline {
            publisher.publish(msg::Empty::default())?;
            let (exec, _) = executor
                .spin_async(SpinOptions::spin_once().timeout(Duration::from_millis(200)))
                .await;
            executor = exec;
        }

        assert!(
            received.load(Ordering::Relaxed) > 0,
            "no message delivered while running on the adopted runtime",
        );
        Ok(())
    }

    /// Adopting a current-thread runtime works for a short non-blocking callback.
    #[tokio::test(flavor = "current_thread")]
    async fn tokio_adopts_current_thread_runtime() -> Result<(), RclrsError> {
        let mut executor = Context::default().create_tokio_executor_on_current_runtime();
        let node = executor.create_node(
            format!("test_tokio_adopt_ct_{}", line!()).start_parameter_services(false),
        )?;
        let qos = QoSProfile::default().reliable().keep_last(10);

        let received = Arc::new(AtomicUsize::new(0));
        let received_cb = Arc::clone(&received);
        let _sub = node.create_subscription::<msg::Empty, _>(
            "tokio_adopt_ct_topic".qos(qos),
            move |_m: msg::Empty| {
                received_cb.fetch_add(1, Ordering::Relaxed);
            },
        )?;
        let publisher = node.create_publisher::<msg::Empty>("tokio_adopt_ct_topic".qos(qos))?;

        let deadline = Instant::now() + Duration::from_secs(10);
        while received.load(Ordering::Relaxed) == 0 && Instant::now() < deadline {
            publisher.publish(msg::Empty::default())?;
            let (exec, _) = executor
                .spin_async(SpinOptions::spin_once().timeout(Duration::from_millis(200)))
                .await;
            executor = exec;
        }

        assert!(
            received.load(Ordering::Relaxed) > 0,
            "no message delivered on an adopted current-thread runtime",
        );
        Ok(())
    }

    /// Blocking `spin()` must not be called from within a Tokio runtime; it panics
    /// with guidance to use `spin_async` instead.
    ///
    /// Uses the adopted constructor so there is no owned runtime to drop inside
    /// this async test (dropping an owned Tokio runtime from within a runtime is
    /// itself a panic, which is exactly why owned mode is not for `#[tokio::main]`).
    #[tokio::test(flavor = "multi_thread")]
    async fn tokio_blocking_spin_within_runtime_panics() {
        let mut executor = Context::default().create_tokio_executor_on_current_runtime();
        let _node = executor
            .create_node(
                format!("test_tokio_spin_panic_{}", line!()).start_parameter_services(false),
            )
            .unwrap();

        // A scary panic message on stderr here is expected; the test asserts the
        // panic happened.
        let panicked = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            executor.spin(SpinOptions::default().timeout(Duration::from_millis(10)));
        }))
        .is_err();
        assert!(
            panicked,
            "blocking spin() inside a Tokio runtime should panic"
        );
    }

    /// A spin with a timeout and no work reports a `Timeout` error, matching the
    /// basic executor's contract (rather than returning silently).
    #[test]
    fn tokio_spin_timeout_reports_error() -> Result<(), RclrsError> {
        let mut executor = Context::default().create_tokio_executor();
        let _node = executor.create_node(
            format!("test_tokio_timeout_{}", line!()).start_parameter_services(false),
        )?;

        let errors = executor.spin(SpinOptions::default().timeout(Duration::from_millis(20)));
        assert!(
            errors.iter().any(|e| matches!(
                e,
                RclrsError::RclError {
                    code: RclReturnCode::Timeout,
                    ..
                }
            )),
            "expected a Timeout error from a timed-out spin, got {errors:?}",
        );
        Ok(())
    }

    /// `only_next_available_work` (spin_once) drains the currently-available work
    /// and returns promptly — it must not be ignored (loop forever) on the Tokio
    /// path. We publish then spin_once until the message is delivered.
    #[test]
    fn tokio_spin_once_processes_available_work() -> Result<(), RclrsError> {
        let mut executor = Context::default().create_tokio_executor();
        let node = executor.create_node(
            format!("test_tokio_spin_once_{}", line!()).start_parameter_services(false),
        )?;
        let qos = QoSProfile::default().reliable().keep_last(10);

        let received = Arc::new(AtomicUsize::new(0));
        let received_cb = Arc::clone(&received);
        let _sub = node.create_subscription::<msg::Empty, _>(
            "tokio_spin_once_topic".qos(qos),
            move |_m: msg::Empty| {
                received_cb.fetch_add(1, Ordering::Relaxed);
            },
        )?;
        let publisher = node.create_publisher::<msg::Empty>("tokio_spin_once_topic".qos(qos))?;

        // Each spin_once waits up to its timeout for work, drains it, and returns;
        // republish to ride out discovery. A wedged/ignored spin_once would never
        // deliver the message and this would time out at the outer deadline.
        let deadline = Instant::now() + Duration::from_secs(10);
        while received.load(Ordering::Relaxed) == 0 && Instant::now() < deadline {
            publisher.publish(msg::Empty::default())?;
            let _ = executor.spin(SpinOptions::spin_once().timeout(Duration::from_millis(200)));
        }

        assert!(
            received.load(Ordering::Relaxed) > 0,
            "spin_once never delivered the message (only_next_available_work ignored?)",
        );
        Ok(())
    }

    /// Regression for strict quiescence: once `spin()` returns, no callback may
    /// still be running. The callback signals the moment it starts (resolving the
    /// until-promise, so spinning is asked to stop *while it runs*) and then
    /// blocks for 400ms. `spin()` must not return until it has finished — proven
    /// by `completed` being set and by the elapsed time exceeding the callback's
    /// duration. Using the start-signal (rather than a fixed sleep) makes the
    /// test robust to discovery/delivery latency.
    #[test]
    fn tokio_spin_waits_for_in_flight_callback() -> Result<(), RclrsError> {
        use futures::channel::oneshot;
        use std::sync::Mutex;

        let mut executor = Context::default().create_tokio_executor();
        let node = executor.create_node(
            format!("test_tokio_quiescence_{}", line!()).start_parameter_services(false),
        )?;
        let qos = QoSProfile::default().reliable().keep_last(10);

        let count = Arc::new(AtomicUsize::new(0));
        let completed = Arc::new(AtomicBool::new(false));
        // The long blocking body runs only once "armed", so discovery stays fast.
        let armed = Arc::new(AtomicBool::new(false));
        // Sender the callback uses to announce that it has started running.
        let start_tx = Arc::new(Mutex::new(None::<oneshot::Sender<()>>));

        let (count_cb, completed_cb, armed_cb, tx_cb) = (
            Arc::clone(&count),
            Arc::clone(&completed),
            Arc::clone(&armed),
            Arc::clone(&start_tx),
        );
        let _sub = node.create_subscription::<msg::Empty, _>(
            "tokio_quiescence_topic".qos(qos),
            move |_m: msg::Empty| {
                count_cb.fetch_add(1, Ordering::Relaxed);
                if armed_cb.swap(false, Ordering::AcqRel) {
                    if let Some(tx) = tx_cb.lock().unwrap().take() {
                        let _ = tx.send(());
                    }
                    std::thread::sleep(Duration::from_millis(400));
                    completed_cb.store(true, Ordering::Release);
                }
            },
        )?;
        let publisher = node.create_publisher::<msg::Empty>("tokio_quiescence_topic".qos(qos))?;

        // Discovery: spin_once (fast callback) until a message lands.
        let deadline = Instant::now() + Duration::from_secs(10);
        while count.load(Ordering::Relaxed) == 0 && Instant::now() < deadline {
            publisher.publish(msg::Empty::default())?;
            let _ = executor.spin(SpinOptions::spin_once().timeout(Duration::from_millis(200)));
        }
        assert!(
            count.load(Ordering::Relaxed) > 0,
            "discovery never delivered"
        );

        // Arm, then spin until the callback *starts* (the promise resolves from
        // inside it). spin() waits for the message and the callback to begin, so
        // there is no fixed-window race; the 10s timeout is just a safety net.
        let (tx, rx) = oneshot::channel();
        *start_tx.lock().unwrap() = Some(tx);
        armed.store(true, Ordering::Release);
        let halt_on_start = executor.commands().run(async move {
            let _ = rx.await;
        });

        publisher.publish(msg::Empty::default())?;
        let start = Instant::now();
        executor.spin(
            SpinOptions::default()
                .until_promise_resolved(halt_on_start)
                .timeout(Duration::from_secs(10)),
        );
        let elapsed = start.elapsed();

        assert!(
            completed.load(Ordering::Acquire),
            "spin() returned while a callback was still running (quiescence violated)",
        );
        assert!(
            elapsed >= Duration::from_millis(350),
            "spin() returned after {elapsed:?}, before the in-flight callback finished",
        );
        Ok(())
    }

    /// Regression for notification combining / no message loss: a burst of messages
    /// published while the executor is NOT spinning must all be delivered once it
    /// resumes (the per-entity `pending` accumulator preserves the count), and the
    /// entity must not wedge.
    #[test]
    fn tokio_burst_while_paused_is_delivered() -> Result<(), RclrsError> {
        const BURST: usize = 20;

        let mut executor = Context::default().create_tokio_executor();
        let node = executor
            .create_node(format!("test_tokio_burst_{}", line!()).start_parameter_services(false))?;
        let qos = QoSProfile::default().reliable().keep_last(100);

        let received = Arc::new(AtomicUsize::new(0));
        let received_cb = Arc::clone(&received);
        let _sub = node.create_subscription::<msg::Empty, _>(
            "tokio_burst_topic".qos(qos),
            move |_m: msg::Empty| {
                received_cb.fetch_add(1, Ordering::Relaxed);
            },
        )?;
        let publisher = node.create_publisher::<msg::Empty>("tokio_burst_topic".qos(qos))?;

        // Discovery: get one message through so pub/sub are matched.
        let deadline = Instant::now() + Duration::from_secs(10);
        while received.load(Ordering::Relaxed) == 0 && Instant::now() < deadline {
            publisher.publish(msg::Empty::default())?;
            let _ = executor.spin(SpinOptions::spin_once().timeout(Duration::from_millis(200)));
        }
        let baseline = received.load(Ordering::Relaxed);

        // Burst while NOT spinning: these fire push callbacks that combine into a
        // single queued `Ready` whose accumulated count is BURST.
        for _ in 0..BURST {
            publisher.publish(msg::Empty::default())?;
        }
        std::thread::sleep(Duration::from_millis(300));

        // Resume: a wedged entity or lost count would leave us short.
        let target = baseline + BURST;
        let deadline = Instant::now() + Duration::from_secs(10);
        while received.load(Ordering::Relaxed) < target && Instant::now() < deadline {
            let _ = executor.spin(SpinOptions::spin_once().timeout(Duration::from_millis(200)));
        }

        assert!(
            received.load(Ordering::Relaxed) >= target,
            "only {} of {} messages delivered after a paused burst (combining lost work or wedged)",
            received.load(Ordering::Relaxed),
            target,
        );
        Ok(())
    }

    /// A panicking callback must not wedge the executor: spin() must still return
    /// (quiescence counters not leaked) and the worker must survive to run other
    /// callbacks. Without panic containment the first spin would hang forever on
    /// quiescence and this test would time out.
    #[test]
    fn tokio_panicking_callback_does_not_wedge() -> Result<(), RclrsError> {
        let mut executor = Context::default().create_tokio_executor();
        let node = executor
            .create_node(format!("test_tokio_panic_{}", line!()).start_parameter_services(false))?;
        let qos = QoSProfile::default().reliable().keep_last(10);

        // A subscription whose callback always panics.
        let _panic_sub = node.create_subscription::<msg::Empty, _>(
            "tokio_panic_topic".qos(qos),
            |_m: msg::Empty| panic!("intentional test panic in a callback"),
        )?;
        let panic_pub = node.create_publisher::<msg::Empty>("tokio_panic_topic".qos(qos))?;

        // A healthy subscription on the same worker — it must still run.
        let healthy = Arc::new(AtomicUsize::new(0));
        let healthy_cb = Arc::clone(&healthy);
        let _healthy_sub = node.create_subscription::<msg::Empty, _>(
            "tokio_healthy_topic".qos(qos),
            move |_m: msg::Empty| {
                healthy_cb.fetch_add(1, Ordering::Relaxed);
            },
        )?;
        let healthy_pub = node.create_publisher::<msg::Empty>("tokio_healthy_topic".qos(qos))?;

        let deadline = Instant::now() + Duration::from_secs(10);
        while healthy.load(Ordering::Relaxed) == 0 && Instant::now() < deadline {
            // Each spin processes the panicking callback (contained) and the
            // healthy one. If quiescence leaked, spin() here would never return.
            let _ = panic_pub.publish(msg::Empty::default());
            let _ = healthy_pub.publish(msg::Empty::default());
            let _ = executor.spin(SpinOptions::spin_once().timeout(Duration::from_millis(200)));
        }

        assert!(
            healthy.load(Ordering::Relaxed) > 0,
            "a panicking callback wedged the worker or spin() quiescence",
        );
        Ok(())
    }

    /// End-to-end: a node-scoped subscription receives messages via the
    /// event-driven path (push callback -> mailbox -> worker task -> callback).
    #[test]
    fn tokio_events_pubsub() -> Result<(), RclrsError> {
        let mut executor = Context::default().create_tokio_executor();
        let node = executor.create_node(
            format!("test_tokio_events_pubsub_{}", line!()).start_parameter_services(false),
        )?;
        let qos = QoSProfile::default().reliable().keep_last(10);

        let publisher = node.create_publisher::<msg::Empty>("tokio_events_topic".qos(qos))?;
        let received = Arc::new(AtomicUsize::new(0));
        let received_cb = Arc::clone(&received);
        let _sub = node.create_subscription::<msg::Empty, _>(
            "tokio_events_topic".qos(qos),
            move |_: msg::Empty| {
                received_cb.fetch_add(1, Ordering::Relaxed);
            },
        )?;

        let deadline = Instant::now() + Duration::from_secs(10);
        while received.load(Ordering::Relaxed) == 0 && Instant::now() < deadline {
            publisher.publish(msg::Empty::default())?;
            executor.spin(SpinOptions::new().timeout(Duration::from_millis(50)));
            std::thread::sleep(Duration::from_millis(20));
        }

        assert!(
            received.load(Ordering::Relaxed) > 0,
            "subscription callback never ran via the event-driven path"
        );
        Ok(())
    }

    /// Callbacks must NOT run before spinning (deferred-execution guarantee that
    /// the spin gate provides).
    #[test]
    fn tokio_events_no_callbacks_before_spin() -> Result<(), RclrsError> {
        let mut executor = Context::default().create_tokio_executor();
        let node = executor.create_node(
            format!("test_tokio_no_early_{}", line!()).start_parameter_services(false),
        )?;
        let qos = QoSProfile::default().reliable().keep_last(10);

        let publisher = node.create_publisher::<msg::Empty>("tokio_no_early_topic".qos(qos))?;
        let received = Arc::new(AtomicUsize::new(0));
        let received_cb = Arc::clone(&received);
        let _sub = node.create_subscription::<msg::Empty, _>(
            "tokio_no_early_topic".qos(qos),
            move |_: msg::Empty| {
                received_cb.fetch_add(1, Ordering::Relaxed);
            },
        )?;

        // Publish and wait WITHOUT spinning; the callback must not run.
        for _ in 0..5 {
            publisher.publish(msg::Empty::default())?;
        }
        std::thread::sleep(Duration::from_millis(300));
        assert_eq!(
            received.load(Ordering::Relaxed),
            0,
            "callback ran before the executor was spun"
        );

        // Now spin and confirm the buffered messages are delivered.
        let deadline = Instant::now() + Duration::from_secs(10);
        while received.load(Ordering::Relaxed) == 0 && Instant::now() < deadline {
            publisher.publish(msg::Empty::default())?;
            executor.spin(SpinOptions::new().timeout(Duration::from_millis(50)));
            std::thread::sleep(Duration::from_millis(20));
        }
        assert!(
            received.load(Ordering::Relaxed) > 0,
            "callback never ran while spinning"
        );
        Ok(())
    }

    /// A dropped subscription must stop firing callbacks (its entity is pruned).
    #[test]
    fn tokio_events_dropped_subscription_stops() -> Result<(), RclrsError> {
        let mut executor = Context::default().create_tokio_executor();
        let node = executor
            .create_node(format!("test_tokio_drop_{}", line!()).start_parameter_services(false))?;
        let qos = QoSProfile::default().reliable().keep_last(10);

        let publisher = node.create_publisher::<msg::Empty>("tokio_drop_topic".qos(qos))?;
        let count = Arc::new(AtomicUsize::new(0));
        let count_cb = Arc::clone(&count);
        let sub = node.create_subscription::<msg::Empty, _>(
            "tokio_drop_topic".qos(qos),
            move |_: msg::Empty| {
                count_cb.fetch_add(1, Ordering::Relaxed);
            },
        )?;

        // Confirm the subscription is delivering.
        let deadline = Instant::now() + Duration::from_secs(10);
        while count.load(Ordering::Relaxed) == 0 && Instant::now() < deadline {
            publisher.publish(msg::Empty::default())?;
            executor.spin(SpinOptions::new().timeout(Duration::from_millis(50)));
            std::thread::sleep(Duration::from_millis(20));
        }
        assert!(
            count.load(Ordering::Relaxed) > 0,
            "subscription never delivered"
        );

        // Drop it, then keep publishing + spinning: the callback must not fire again.
        drop(sub);
        let after_drop = count.load(Ordering::Relaxed);
        for _ in 0..10 {
            publisher.publish(msg::Empty::default())?;
            executor.spin(SpinOptions::new().timeout(Duration::from_millis(50)));
        }
        assert_eq!(
            count.load(Ordering::Relaxed),
            after_drop,
            "callback fired after the subscription was dropped"
        );
        Ok(())
    }

    /// End-to-end service round-trip driven entirely by the event-driven executor.
    #[test]
    fn tokio_events_service_roundtrip() -> Result<(), RclrsError> {
        let mut executor = Context::default().create_tokio_executor();
        let node = executor.create_node(
            format!("test_tokio_events_service_{}", line!()).start_parameter_services(false),
        )?;

        let _service = node.create_service::<test_msgs::srv::Empty, _>(
            "tokio_events_service",
            |_request: test_msgs::srv::Empty_Request| test_msgs::srv::Empty_Response::default(),
        )?;
        let client = node.create_client::<test_msgs::srv::Empty>("tokio_events_service")?;

        let deadline = Instant::now() + Duration::from_secs(10);
        while !client.service_is_ready()? {
            assert!(Instant::now() < deadline, "service never became ready");
            std::thread::sleep(Duration::from_millis(20));
        }

        let response: Promise<test_msgs::srv::Empty_Response> =
            client.call(test_msgs::srv::Empty_Request::default())?;
        let (mut response, notice) = executor.commands().create_notice(response);
        executor.spin(
            SpinOptions::new()
                .until_promise_resolved(notice)
                .timeout(Duration::from_secs(5)),
        );

        assert!(
            response.try_recv().ok().flatten().is_some(),
            "client never received the service response via the event-driven path"
        );
        Ok(())
    }

    /// Timers fire on the event-driven executor.
    #[test]
    fn tokio_events_timer_fires() -> Result<(), RclrsError> {
        let mut executor = Context::default().create_tokio_executor();
        let node = executor.create_node(
            format!("test_tokio_events_timer_{}", line!()).start_parameter_services(false),
        )?;

        let count = Arc::new(AtomicUsize::new(0));
        let count_cb = Arc::clone(&count);
        let _timer = node.create_timer_repeating(Duration::from_millis(10), move || {
            count_cb.fetch_add(1, Ordering::Relaxed);
        })?;

        executor.spin(SpinOptions::new().timeout(Duration::from_millis(300)));

        let fired = count.load(Ordering::Relaxed);
        assert!(
            fired >= 3,
            "timer fired only {fired} times in ~300ms (expected several)"
        );
        Ok(())
    }

    /// Worker-scoped subscription + `listen_until` activity listener on the
    /// event-driven executor.
    #[test]
    fn tokio_events_worker() -> Result<(), RclrsError> {
        let mut executor = Context::default().create_tokio_executor();
        let node = executor.create_node(
            format!("test_tokio_worker_{}", line!()).start_parameter_services(false),
        )?;

        let worker = node.create_worker::<usize>(0);
        let _sub = worker.create_subscription(
            "tokio_worker_topic",
            |payload: &mut usize, _msg: msg::Empty| {
                *payload += 1;
            },
        )?;
        let promise = worker.listen_until(|payload: &mut usize| (*payload > 0).then_some(*payload));

        let publisher = node.create_publisher::<msg::Empty>("tokio_worker_topic")?;
        let stop = Arc::new(AtomicBool::new(false));
        let stop_pub = Arc::clone(&stop);
        let pub_thread = std::thread::spawn(move || {
            while !stop_pub.load(Ordering::Acquire) {
                let _ = publisher.publish(msg::Empty::default());
                std::thread::sleep(Duration::from_millis(10));
            }
        });

        let (mut promise, notice) = executor.commands().create_notice(promise);
        executor.spin(
            SpinOptions::new()
                .until_promise_resolved(notice)
                .timeout(Duration::from_secs(5)),
        );
        stop.store(true, Ordering::Release);
        pub_thread.join().unwrap();

        assert!(
            promise.try_recv().ok().flatten().is_some(),
            "worker subscription / activity listener never fired on the event-driven executor"
        );
        Ok(())
    }

    /// A node with parameter services enabled drives cleanly on the executor.
    #[test]
    fn tokio_events_node_with_parameter_services() -> Result<(), RclrsError> {
        let mut executor = Context::default().create_tokio_executor();
        let _node = executor.create_node(&format!("test_tokio_paramsvc_{}", line!()))?;
        let errors = executor.spin(SpinOptions::new().timeout(Duration::from_millis(200)));
        // A bare-timeout spin reports a `Timeout` error (matching the basic
        // executor); assert nothing *other* than that was produced.
        assert!(
            errors.iter().all(|e| matches!(
                e,
                RclrsError::RclError {
                    code: RclReturnCode::Timeout,
                    ..
                }
            )),
            "spinning a node with parameter services produced unexpected errors: {errors:?}"
        );
        Ok(())
    }

    /// Async tasks run on the Tokio runtime (would panic on the basic executor).
    #[test]
    fn tokio_async_task_runs() {
        let mut executor = Context::default().create_tokio_executor();
        let _node = executor
            .create_node(&format!("test_tokio_async_task_{}", line!()))
            .unwrap();

        let done = Arc::new(AtomicBool::new(false));
        let done_clone = Arc::clone(&done);

        let promise = executor.commands().run(async move {
            tokio::time::sleep(Duration::from_millis(1)).await;
            done_clone.store(true, Ordering::Release);
        });

        let (_, notice) = executor.commands().create_notice(promise);
        executor
            .spin(
                SpinOptions::new()
                    .until_promise_resolved(notice)
                    .timeout(Duration::from_secs(5)),
            )
            .first_error()
            .unwrap();

        assert!(done.load(Ordering::Acquire));
    }

    /// A 1 kHz timer should fire close to its ideal rate with the shared scheduler.
    #[test]
    fn tokio_high_rate_timer() -> Result<(), RclrsError> {
        let mut executor = Context::default().create_tokio_executor();
        let node = executor.create_node(
            format!("test_tokio_high_rate_{}", line!()).start_parameter_services(false),
        )?;

        let count = Arc::new(AtomicUsize::new(0));
        let count_cb = Arc::clone(&count);
        let _timer = node.create_timer_repeating(Duration::from_millis(1), move || {
            count_cb.fetch_add(1, Ordering::Relaxed);
        })?;

        executor.spin(SpinOptions::new().timeout(Duration::from_secs(1)));

        let fired = count.load(Ordering::Relaxed);
        assert!(
            fired >= 800,
            "1 kHz timer fired only {fired} times in ~1s (expected >= 800)"
        );
        Ok(())
    }

    /// Several timers at different rates each fire from one shared scheduler.
    #[test]
    fn tokio_multi_rate_timers() -> Result<(), RclrsError> {
        let mut executor = Context::default().create_tokio_executor();
        let node = executor.create_node(
            format!("test_tokio_multi_rate_{}", line!()).start_parameter_services(false),
        )?;

        let fast = Arc::new(AtomicUsize::new(0));
        let slow = Arc::new(AtomicUsize::new(0));
        let fast_cb = Arc::clone(&fast);
        let slow_cb = Arc::clone(&slow);

        let _t1 = node.create_timer_repeating(Duration::from_millis(5), move || {
            fast_cb.fetch_add(1, Ordering::Relaxed);
        })?;
        let _t2 = node.create_timer_repeating(Duration::from_millis(20), move || {
            slow_cb.fetch_add(1, Ordering::Relaxed);
        })?;

        executor.spin(SpinOptions::new().timeout(Duration::from_millis(500)));

        let fast_fired = fast.load(Ordering::Relaxed);
        let slow_fired = slow.load(Ordering::Relaxed);
        assert!(
            fast_fired >= 60,
            "fast timer fired only {fast_fired} times in ~500ms (expected ~100)"
        );
        assert!(
            slow_fired >= 15,
            "slow timer fired only {slow_fired} times in ~500ms (expected ~25)"
        );
        assert!(
            fast_fired > slow_fired * 2,
            "fast timer ({fast_fired}) did not outpace slow ({slow_fired})"
        );
        Ok(())
    }

    /// Cancelled timers stop firing.
    #[test]
    fn tokio_timer_cancel_stops() -> Result<(), RclrsError> {
        let mut executor = Context::default().create_tokio_executor();
        let node = executor.create_node(
            format!("test_tokio_timer_cancel_{}", line!()).start_parameter_services(false),
        )?;

        let count = Arc::new(AtomicUsize::new(0));
        let count_cb = Arc::clone(&count);
        let timer = node.create_timer_repeating(Duration::from_millis(5), move || {
            count_cb.fetch_add(1, Ordering::Relaxed);
        })?;

        executor.spin(SpinOptions::new().timeout(Duration::from_millis(100)));
        assert!(
            count.load(Ordering::Relaxed) > 0,
            "timer never fired before cancel"
        );

        timer.cancel()?;
        let after_cancel = count.load(Ordering::Relaxed);
        executor.spin(SpinOptions::new().timeout(Duration::from_millis(100)));
        assert_eq!(
            count.load(Ordering::Relaxed),
            after_cancel,
            "timer kept firing after cancel"
        );
        Ok(())
    }

    /// A slow callback causes dropped ticks, not unbounded pending growth.
    #[test]
    fn tokio_slow_timer_callback_drops_ticks() -> Result<(), RclrsError> {
        let mut executor = Context::default().create_tokio_executor();
        let node = executor.create_node(
            format!("test_tokio_slow_timer_{}", line!()).start_parameter_services(false),
        )?;

        let count = Arc::new(AtomicUsize::new(0));
        let count_cb = Arc::clone(&count);
        let _timer = node.create_timer_repeating(Duration::from_millis(1), move || {
            count_cb.fetch_add(1, Ordering::Relaxed);
            std::thread::sleep(Duration::from_millis(10));
        })?;

        executor.spin(SpinOptions::new().timeout(Duration::from_millis(200)));

        let fired = count.load(Ordering::Relaxed);
        assert!(
            fired < 150,
            "slow callback allowed {fired} fires in 200ms (pending burst?)"
        );
        assert!(fired >= 5, "timer never fired at all ({fired})");
        Ok(())
    }

    /// Steady- and system-clock timers both fire on the shared scheduler.
    #[test]
    fn tokio_timer_steady_and_system_clocks() -> Result<(), RclrsError> {
        use crate::IntoTimerOptions;

        let mut executor = Context::default().create_tokio_executor();
        let node = executor.create_node(
            format!("test_tokio_timer_clocks_{}", line!()).start_parameter_services(false),
        )?;

        let steady_count = Arc::new(AtomicUsize::new(0));
        let system_count = Arc::new(AtomicUsize::new(0));
        let steady_cb = Arc::clone(&steady_count);
        let system_cb = Arc::clone(&system_count);

        let _steady =
            node.create_timer_repeating(Duration::from_millis(10).steady_time(), move || {
                steady_cb.fetch_add(1, Ordering::Relaxed);
            })?;
        let _system =
            node.create_timer_repeating(Duration::from_millis(10).system_time(), move || {
                system_cb.fetch_add(1, Ordering::Relaxed);
            })?;

        executor.spin(SpinOptions::new().timeout(Duration::from_millis(200)));

        assert!(
            steady_count.load(Ordering::Relaxed) >= 5,
            "steady-clock timer did not fire"
        );
        assert!(
            system_count.load(Ordering::Relaxed) >= 5,
            "system-clock timer did not fire"
        );
        Ok(())
    }

    /// A cancelled timer fires again after reset().
    #[test]
    fn tokio_timer_reset_after_cancel() -> Result<(), RclrsError> {
        let mut executor = Context::default().create_tokio_executor();
        let node = executor.create_node(
            format!("test_tokio_timer_reset_cancel_{}", line!()).start_parameter_services(false),
        )?;

        let count = Arc::new(AtomicUsize::new(0));
        let count_cb = Arc::clone(&count);
        let timer = node.create_timer_repeating(Duration::from_millis(5), move || {
            count_cb.fetch_add(1, Ordering::Relaxed);
        })?;

        executor.spin(SpinOptions::new().timeout(Duration::from_millis(100)));
        assert!(
            count.load(Ordering::Relaxed) > 0,
            "timer never fired before cancel"
        );

        timer.cancel()?;
        let after_cancel = count.load(Ordering::Relaxed);
        executor.spin(SpinOptions::new().timeout(Duration::from_millis(100)));
        assert_eq!(
            count.load(Ordering::Relaxed),
            after_cancel,
            "timer kept firing after cancel"
        );

        timer.reset()?;
        executor.spin(SpinOptions::new().timeout(Duration::from_millis(100)));
        assert!(
            count.load(Ordering::Relaxed) > after_cancel,
            "timer did not fire again after reset"
        );
        Ok(())
    }

    /// reset() on an active timer keeps it firing without stopping it or
    /// double-scheduling a burst.
    #[test]
    fn tokio_timer_reset_keeps_firing() -> Result<(), RclrsError> {
        let mut executor = Context::default().create_tokio_executor();
        let node = executor.create_node(
            format!("test_tokio_timer_reset_reseed_{}", line!()).start_parameter_services(false),
        )?;

        let count = Arc::new(AtomicUsize::new(0));
        let count_cb = Arc::clone(&count);
        let timer = node.create_timer_repeating(Duration::from_millis(20), move || {
            count_cb.fetch_add(1, Ordering::Relaxed);
        })?;

        executor.spin(SpinOptions::new().timeout(Duration::from_millis(100)));
        let before_reset = count.load(Ordering::Relaxed);
        assert!(before_reset > 0, "timer never fired before reset");

        timer.reset()?;
        executor.spin(SpinOptions::new().timeout(Duration::from_millis(200)));
        let after = count.load(Ordering::Relaxed) - before_reset;
        assert!(
            after >= 4,
            "timer stopped firing after reset ({after} in ~200ms, expected ~10)"
        );
        assert!(
            after <= 20,
            "reset() caused a burst ({after} in ~200ms, expected ~10)"
        );
        Ok(())
    }

    /// Repeating timers stay near their ideal rate over several seconds.
    #[test]
    fn tokio_timer_no_drift() -> Result<(), RclrsError> {
        let mut executor = Context::default().create_tokio_executor();
        let node = executor.create_node(
            format!("test_tokio_timer_no_drift_{}", line!()).start_parameter_services(false),
        )?;

        let count = Arc::new(AtomicUsize::new(0));
        let count_cb = Arc::clone(&count);
        let _timer = node.create_timer_repeating(Duration::from_millis(10), move || {
            count_cb.fetch_add(1, Ordering::Relaxed);
        })?;

        executor.spin(SpinOptions::new().timeout(Duration::from_secs(2)));

        let fired = count.load(Ordering::Relaxed);
        assert!(
            fired >= 160,
            "10 ms timer drifted low: {fired} fires in ~2s (expected ~200)"
        );
        assert!(
            fired <= 240,
            "10 ms timer drifted high: {fired} fires in ~2s (expected ~200)"
        );
        Ok(())
    }
}

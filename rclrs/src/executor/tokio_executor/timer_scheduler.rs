//! Shared timer scheduler for the Tokio executor.
//!
//! One min-heap keyed by next deadline drives all timers. A dedicated OS thread
//! waits on a [`Condvar`] until the earliest deadline. Pacing is decoupled from
//! execution: the scheduler enqueues `Ready` messages and advances deadlines
//! without waiting for worker callbacks.
//!
//! ## Design
//!
//! The waiter is a plain OS thread blocking on a [`std::sync::Condvar`], not a
//! Tokio task. That buys precise pacing (a direct `wait_timeout`), isolation from
//! the worker tasks, and independence from the Tokio timer driver, at the cost
//! of having to guard the blocking wait by hand.
//!
//! The waiter computes its sleep from the heap under the state lock, then
//! *releases* that lock before blocking, so producers (`register` / `reschedule`
//! / `cancel`) stay responsive instead of being held off for the whole sleep.
//!
//! There is a possibility for a wakeup to be lost. A producer can mutate the heap and
//! signal in the gap between the waiter "deciding to sleep" and "actually
//! sleeping", and a bare condvar only wakes a thread that is *already* parked, so
//! the signal would be missed and the waiter would oversleep.
//!
//! [`TimerSchedulerState::generation`] closes that window. It is a version stamp
//! bumped on every change the waiter cares about (see [`bump_generation`]); the
//! waiter snapshots it under the state lock and, immediately before blocking,
//! re-checks it under the condvar's own lock (a separate, tiny lock so producers
//! never contend on the state lock during the wait). If it moved, the waiter
//! skips the wait and recomputes against the fresh state.
//!
//! This generation dance is the hand-rolled equivalent of what an async design
//! gets for free from a `tokio::select!` over a `sleep` and a `watch`/`Notify`:
//! the lost-wakeup is universal to notify-style signaling, but a *synchronous*
//! condvar is what forces us to reconstruct the "something changed, re-evaluate"
//! coalescing ourselves rather than leaning on the runtime's poll model.
//!
//! ## Practical limits (Linux, measured)
//!
//! - Single timer: ~16 kHz before worker round-trip latency caps throughput.
//!   Recommended ≤ ~5 kHz with margin.
//! - Many timers: hundreds of kHz aggregate on one worker (empty callbacks).
//! - Heavy callbacks serialize: keep `rate × work_us` well below 1e6 µs/s.

use std::{
    cmp::Ordering as CmpOrdering,
    collections::{BinaryHeap, HashMap},
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Condvar, Mutex,
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

use crate::{
    rcl_bindings::{
        rcl_timer_get_period, rcl_timer_get_time_until_next_call, rcl_timer_is_canceled,
        rcl_timer_is_ready, rcl_timer_t,
    },
    ToResult,
};

use super::EntityDispatch;

/// Drives every timer owned by a single Tokio executor.
///
/// There is one `TimerScheduler` per executor. It owns the shared
/// [`TimerSchedulerState`] (the deadline heap and the timer table) and a
/// dedicated OS thread that sleeps until the earliest deadline and then enqueues
/// ticks on the owning workers' mailboxes. Timers register via [`register`] and
/// receive a [`TimerSchedulerNotify`] handle they use to reschedule/cancel
/// themselves; dropping the scheduler stops and joins the thread.
///
/// [`register`]: TimerScheduler::register
pub(crate) struct TimerScheduler {
    /// Heap + timer table, shared with the waiter thread and every notify handle.
    state: Arc<Mutex<TimerSchedulerState>>,
    /// Wakeup/shutdown signaling shared with the waiter thread.
    wake: Arc<SharedWake>,
    /// The waiter thread; `Option` so [`Drop`] can `take()` and join it.
    thread_waiter: Option<JoinHandle<()>>,
}

impl TimerScheduler {
    /// Create the scheduler and spawn its waiter thread (initially idle, with no
    /// timers registered).
    pub(crate) fn new() -> Self {
        let state = Arc::new(Mutex::new(TimerSchedulerState::default()));
        let wake = Arc::new(SharedWake::default());

        let thread_waiter = Some(spawn_thread_waiter(Arc::clone(&state), Arc::clone(&wake)));

        Self {
            state,
            wake,
            thread_waiter,
        }
    }

    /// Register a timer with the scheduler and arm its first deadline.
    ///
    /// Reads the timer's period and time-until-next-call from rcl, inserts an
    /// entry plus a heap node, wakes the waiter thread, and returns a
    /// [`TimerSchedulerNotify`] the caller stores on the timer so later
    /// reset/cancel/drop can update the schedule.
    pub(crate) fn register(&self, reg: TimerRegistration) -> TimerSchedulerNotify {
        let period = read_period(&reg.rcl_timer);
        let deadline = next_deadline(&reg.rcl_timer);

        let mut state = self.state.lock().unwrap();
        let timer_id = state.allocate_timer_id();

        // Store the timer
        state
            .entries
            .insert(timer_id, TimerEntry::new(&reg, period, deadline));

        // Store the next deadline
        state.heap.push(HeapKey { deadline, timer_id });
        bump_generation(&mut state, &self.wake);

        TimerSchedulerNotify {
            rcl_timer: reg.rcl_timer,
            timer_id,
            state: Arc::clone(&self.state),
            wake: Arc::clone(&self.wake),
        }
    }
}

impl Drop for TimerScheduler {
    /// Signal shutdown, wake the waiter thread, and join it.
    fn drop(&mut self) {
        self.wake.shutdown.store(true, Ordering::Release);
        self.wake.condvar.notify_all();
        if let Some(handle) = self.thread_waiter.take() {
            let _ = handle.join();
        }
    }
}

/// Per-timer handle into the scheduler, stored on the timer itself.
///
/// Lets a timer's [`reset`][crate::TimerState::reset],
/// [`cancel`][crate::TimerState::cancel], and `Drop` update the shared schedule
/// (re-arm, remove from the heap, or drop the entry) and wake the waiter thread.
#[derive(Clone)]
pub(crate) struct TimerSchedulerNotify {
    /// The timer this handle controls, used to re-read its next deadline.
    rcl_timer: Arc<Mutex<rcl_timer_t>>,

    /// Key into [`TimerSchedulerState::entries`].
    timer_id: u64,

    /// Shared scheduler state (heap + table).
    state: Arc<Mutex<TimerSchedulerState>>,

    /// Wakeup signaling for the waiter thread.
    wake: Arc<SharedWake>,
}

impl TimerSchedulerNotify {
    /// Re-seed this timer's deadline from rcl and wake the waiter.
    ///
    /// Used by `reset()` to re-arm a (possibly cancelled) timer: it recomputes
    /// the deadline, pushes a fresh heap node, and bumps the generation.
    pub(crate) fn reschedule(&self) {
        let deadline = next_deadline(&self.rcl_timer);
        let mut state = self.state.lock().unwrap();
        if state.schedule_at(self.timer_id, deadline) {
            bump_generation(&mut state, &self.wake);
        }
    }

    /// Remove this timer from the wait heap until it is rescheduled.
    ///
    /// Used by `cancel()`: the entry is kept (so a later `reset()` can re-arm it)
    /// but marked out-of-heap so any stale heap node is ignored.
    pub(crate) fn unschedule(&self) {
        let mut state = self.state.lock().unwrap();
        let Some(entry) = state.entries.get_mut(&self.timer_id) else {
            return;
        };
        entry.in_heap = false;
        bump_generation(&mut state, &self.wake);
    }

    /// Drop this timer's entry entirely. Called when the timer is dropped so a
    /// cancelled (and therefore heap-less) timer does not leak its entry.
    pub(crate) fn remove(&self) {
        let mut state = self.state.lock().unwrap();
        if state.entries.remove(&self.timer_id).is_some() {
            bump_generation(&mut state, &self.wake);
        }
    }
}

/// Registration inputs cloned from a worker entity at `add_to_wait_set` time.
///
/// Carries everything the scheduler needs to pace a timer and deliver its ticks
/// to the owning worker.
pub(crate) struct TimerRegistration {
    /// The rcl timer to pace.
    pub rcl_timer: Arc<Mutex<rcl_timer_t>>,

    /// Readiness dispatch shared with the owning worker entity; ticks are
    /// delivered through it (notification-combining + outstanding accounting).
    pub dispatch: Arc<EntityDispatch>,

    /// False once the entity is being torn down; tells the scheduler to drop it.
    pub in_use: Arc<AtomicBool>,
}

/// Shared scheduler state guarded by a single mutex.
#[derive(Default)]
struct TimerSchedulerState {
    /// Min-heap (via [`HeapKey`]'s reversed ordering) of next deadlines. May hold
    /// stale nodes; staleness is detected against [`TimerEntry`] when popped.
    heap: BinaryHeap<HeapKey>,

    /// All registered timers, keyed by id.
    entries: HashMap<u64, TimerEntry>,

    /// Monotonic id allocator for new timers.
    next_timer_id: u64,

    /// Version stamp of the state, bumped on every change the waiter cares about
    /// so it can re-evaluate and not lose a wakeup. See the module-level doc.
    generation: u64,
}

impl TimerSchedulerState {
    /// Allocate a fresh, unique timer id.
    fn allocate_timer_id(&mut self) -> u64 {
        let id = self.next_timer_id;
        self.next_timer_id += 1;
        id
    }

    /// Arm `timer_id` to fire at `deadline`: update its entry and push a heap
    /// node for it. Returns `false` (a no-op) if the timer has no entry. Does not
    /// bump the generation — the caller wakes the waiter thread when appropriate.
    fn schedule_at(&mut self, timer_id: u64, deadline: Instant) -> bool {
        let Some(entry) = self.entries.get_mut(&timer_id) else {
            return false;
        };
        entry.deadline = deadline;
        entry.in_heap = true;
        self.heap.push(HeapKey { deadline, timer_id });
        true
    }
}

/// The scheduler's record for one timer.
struct TimerEntry {
    /// The rcl timer, re-read each time it becomes due.
    rcl_timer: Arc<Mutex<rcl_timer_t>>,

    /// Cached period, used to advance the deadline without an rcl round-trip.
    period: Duration,

    /// The authoritative next deadline for this timer.
    deadline: Instant,

    /// Readiness dispatch to the owning worker entity: ticks are delivered
    /// through it (notification-combining + outstanding accounting).
    dispatch: Arc<EntityDispatch>,

    /// False once the entity is torn down.
    in_use: Arc<AtomicBool>,

    /// Whether a heap node with `deadline` is still authoritative. Lets the
    /// scheduler invalidate heap nodes lazily (mark, don't remove).
    in_heap: bool,
}

impl TimerEntry {
    /// Build an entry for a freshly registered timer, armed at `deadline` and
    /// marked present in the heap.
    fn new(reg: &TimerRegistration, period: Duration, deadline: Instant) -> Self {
        Self {
            rcl_timer: Arc::clone(&reg.rcl_timer),
            period,
            deadline,
            dispatch: Arc::clone(&reg.dispatch),
            in_use: Arc::clone(&reg.in_use),
            in_heap: true,
        }
    }
}

/// A node in the deadline heap: the timer id tagged with the deadline it was
/// pushed for. Compared so the heap behaves as a min-heap on `deadline`.
struct HeapKey {
    deadline: Instant,
    timer_id: u64,
}

impl Clone for HeapKey {
    fn clone(&self) -> Self {
        Self {
            deadline: self.deadline,
            timer_id: self.timer_id,
        }
    }
}

impl PartialEq for HeapKey {
    fn eq(&self, other: &Self) -> bool {
        self.deadline == other.deadline && self.timer_id == other.timer_id
    }
}

impl Eq for HeapKey {}

impl PartialOrd for HeapKey {
    fn partial_cmp(&self, other: &Self) -> Option<CmpOrdering> {
        Some(self.cmp(other))
    }
}

impl Ord for HeapKey {
    fn cmp(&self, other: &Self) -> CmpOrdering {
        // Earliest deadline at the top of the max-heap.
        other
            .deadline
            .cmp(&self.deadline)
            .then_with(|| other.timer_id.cmp(&self.timer_id))
    }
}

/// Wakeup and shutdown signaling between producers (register/notify/drop) and
/// the waiter thread.
#[derive(Default)]
struct SharedWake {
    /// Set on drop; the waiter thread exits when it observes this.
    shutdown: AtomicBool,

    /// Mirror of [`TimerSchedulerState::generation`] used for the condvar wait.
    generation: Mutex<u64>,

    /// Signaled on every generation bump and on shutdown.
    condvar: Condvar,
}

/// Snapshot of an rcl timer's state, read together under the timer lock.
struct TimerRclStatus {
    canceled: bool,
    ready: bool,
    time_until_next: Duration,
}

/// Bump the generation (under the state lock) and signal the waiter thread.
///
/// The generation mirror in [`SharedWake`] lets the waiter detect that state
/// changed between releasing the state lock and starting its condvar wait,
/// avoiding a lost wakeup.
fn bump_generation(state: &mut TimerSchedulerState, wake: &SharedWake) {
    state.generation = state.generation.wrapping_add(1);
    *wake.generation.lock().unwrap() = state.generation;
    wake.condvar.notify_all();
}

/// Spawn the named waiter thread that drives the scheduler.
fn spawn_thread_waiter(
    state: Arc<Mutex<TimerSchedulerState>>,
    wake: Arc<SharedWake>,
) -> JoinHandle<()> {
    thread::Builder::new()
        .name("rclrs-timer-scheduler".into())
        .spawn(move || run_thread_waiter(state, wake))
        .expect("failed to spawn timer scheduler thread")
}

/// The waiter thread's main loop: sleep until the earliest deadline (or until
/// woken by a generation bump/shutdown), then fire due timers.
fn run_thread_waiter(state: Arc<Mutex<TimerSchedulerState>>, wake: Arc<SharedWake>) {
    #[cfg(windows)]
    let _timer_resolution = WindowsTimerResolution::raise();

    loop {
        if wake.shutdown.load(Ordering::Acquire) {
            return;
        }

        let (next_deadline, generation) = {
            let inner = state.lock().unwrap();
            (inner.heap.peek().map(|k| k.deadline), inner.generation)
        };

        // Wait under the generation lock, guarding against a lost wakeup: if a
        // producer bumped the generation between our (released) state read and
        // acquiring this lock, skip the wait and re-evaluate immediately.
        match next_deadline {
            None => {
                let guard = wake.generation.lock().unwrap();

                if *guard == generation {
                    let _ = wake.condvar.wait(guard).ok();
                }
            }
            Some(deadline) => {
                let now = Instant::now();

                if deadline > now {
                    let guard = wake.generation.lock().unwrap();

                    if *guard != generation {
                        continue;
                    }

                    let (_guard, timeout) =
                        wake.condvar.wait_timeout(guard, deadline - now).unwrap();

                    if !timeout.timed_out() {
                        continue;
                    }
                }
                process_due_timers(&state, &wake, generation);
            }
        }
    }
}

/// Drain every timer whose deadline has passed, enqueueing ticks and advancing
/// (or removing) each entry.
///
/// For each due timer it re-reads rcl to decide what to do: drop a torn-down
/// timer, leave a cancelled one as a tombstone, re-arm one that is not actually
/// ready yet, drop a tick when the worker is behind, or enqueue a tick and
/// advance by one period. `seen_generation` lets it bail out if the state was
/// mutated concurrently.
fn process_due_timers(
    state: &Arc<Mutex<TimerSchedulerState>>,
    wake: &Arc<SharedWake>,
    seen_generation: u64,
) {
    let now = Instant::now();
    loop {
        if wake.shutdown.load(Ordering::Acquire) {
            return;
        }

        let head = {
            let inner = state.lock().unwrap();
            inner.heap.peek().cloned()
        };

        let Some(head) = head else {
            break;
        };

        if head.deadline > now {
            break;
        }

        let mut reschedule = None;
        let mut remove = false;

        {
            let mut inner = state.lock().unwrap();
            if inner.generation != seen_generation {
                break;
            }

            inner.heap.pop();
            let Some(entry) = inner.entries.get_mut(&head.timer_id) else {
                continue;
            };

            if !entry.in_heap || entry.deadline != head.deadline {
                continue;
            }
            entry.in_heap = false;

            if !entry.in_use.load(Ordering::Acquire) {
                remove = true;
            } else {
                let status = read_timer_status(&entry.rcl_timer);
                if status.canceled {
                    // Tombstone: keep the entry (out of the heap) so reset() can re-arm.
                } else if !status.ready {
                    reschedule = Some(Instant::now() + status.time_until_next);
                } else if entry.dispatch.is_scheduled() {
                    // Worker is behind: drop this tick without growing `pending`.
                    reschedule = Some(entry.deadline + entry.period);
                } else if !entry.dispatch.notify(1) {
                    // Worker's mailbox is closed: drop the timer.
                    remove = true;
                } else {
                    reschedule = Some(entry.deadline + entry.period);
                }
            }

            if remove {
                inner.entries.remove(&head.timer_id);
            } else if let Some(dl) = reschedule {
                inner.schedule_at(head.timer_id, dl);
            }
        }
    }
}

/// Read the timer's configured period, defaulting to 1 ms on error.
fn read_period(rcl_timer: &Arc<Mutex<rcl_timer_t>>) -> Duration {
    let timer = rcl_timer.lock().unwrap();
    let mut period_ns: i64 = 0;
    // SAFETY: handle valid and locked.
    let ret = unsafe { rcl_timer_get_period(&*timer, &mut period_ns) };
    ret.ok()
        .ok()
        .map(|()| Duration::from_nanos(period_ns.max(0) as u64))
        .unwrap_or(Duration::from_millis(1))
}

/// Read just the time until the timer's next call (see [`read_timer_status`]).
fn read_time_until_next_call(rcl_timer: &Arc<Mutex<rcl_timer_t>>) -> Duration {
    read_timer_status(rcl_timer).time_until_next
}

/// The next absolute deadline for `rcl_timer`, derived from its current
/// time-until-next-call. Used when (re)arming a timer.
fn next_deadline(rcl_timer: &Arc<Mutex<rcl_timer_t>>) -> Instant {
    Instant::now() + read_time_until_next_call(rcl_timer)
}

/// Read cancellation/readiness/time-until-next together under the timer lock.
///
/// On error each field falls back conservatively: treated as canceled, not
/// ready, and a 50 ms retry delay so a transiently failing timer is re-polled
/// rather than busy-looped.
fn read_timer_status(rcl_timer: &Arc<Mutex<rcl_timer_t>>) -> TimerRclStatus {
    let timer = rcl_timer.lock().unwrap();

    let mut canceled = false;
    // SAFETY: handle valid and locked.
    let canceled = unsafe { rcl_timer_is_canceled(&*timer, &mut canceled) }
        .ok()
        .map(|()| canceled)
        .unwrap_or(true);

    let mut ready = false;
    // SAFETY: handle valid and locked.
    let ready = unsafe { rcl_timer_is_ready(&*timer, &mut ready) }
        .ok()
        .map(|()| ready)
        .unwrap_or(false);

    let mut value: i64 = 0;
    // SAFETY: handle valid and locked.
    let time_until_next =
        match unsafe { rcl_timer_get_time_until_next_call(&*timer, &mut value) }.ok() {
            Ok(()) if value > 0 => Duration::from_nanos(value as u64),
            Ok(()) => Duration::ZERO,
            Err(_) => Duration::from_millis(50),
        };

    TimerRclStatus {
        canceled,
        ready,
        time_until_next,
    }
}

/// On Windows the default scheduler tick is ~15.6 ms; request 1 ms for the
/// lifetime of the timer scheduler thread.
#[cfg(windows)]
struct WindowsTimerResolution {
    active: bool,
}

#[cfg(windows)]
impl WindowsTimerResolution {
    fn raise() -> Self {
        extern "system" {
            fn timeBeginPeriod(u_period: u32) -> u32;
        }
        let active = unsafe { timeBeginPeriod(1) } == 0;
        Self { active }
    }
}

#[cfg(windows)]
impl Drop for WindowsTimerResolution {
    fn drop(&mut self) {
        if self.active {
            extern "system" {
                fn timeEndPeriod(u_period: u32) -> u32;
            }
            unsafe {
                timeEndPeriod(1);
            }
        }
    }
}

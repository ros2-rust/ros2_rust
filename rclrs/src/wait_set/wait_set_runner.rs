use futures::channel::{
    mpsc::{unbounded, UnboundedReceiver, UnboundedSender},
    oneshot::channel,
};

use std::{
    any::Any,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    time::{Duration, Instant},
};

use crate::{
    log_debug, log_fatal, ActivityListenerCallback, Context, ExecutorWorkerOptions, GuardCondition,
    PayloadTask, Promise, RclReturnCode, RclrsError, WaitSet, Waitable, WeakActivityListener,
};

/// This is a utility class that executors can use to easily run and manage
/// their wait set.
pub struct WaitSetRunner {
    wait_set: WaitSet,
    waitable_sender: UnboundedSender<Waitable>,
    waitable_receiver: UnboundedReceiver<Waitable>,
    task_sender: UnboundedSender<PayloadTask>,
    task_receiver: UnboundedReceiver<PayloadTask>,
    activity_listeners: Arc<Mutex<Vec<WeakActivityListener>>>,
    guard_condition: Arc<GuardCondition>,
    payload: Box<dyn Any + Send>,
}

/// These are the conditions used by the [`WaitSetRunner`] to determine when it
/// needs to halt.
#[derive(Clone, Debug)]
pub struct WaitSetRunConditions {
    /// Only perform the next available work. This is similar to spin_once in
    /// rclcpp and rclpy.
    ///
    /// To only process work that is immediately available without waiting at all,
    /// set a timeout of zero.
    pub only_next_available_work: bool,
    /// Stop spinning once this instant in time is reached.
    pub stop_time: Option<Instant>,
    /// Use this to check [`Context::ok`] to make sure that the context is still
    /// valid. When the context is invalid, the executor runtime should stop
    /// spinning.
    pub context: Context,
    /// Halt trigger that gets set by [`ExecutorCommands`][1].
    ///
    /// [1]: crate::ExecutorCommands
    pub halt_spinning: Arc<AtomicBool>,
}

impl WaitSetRunner {
    /// Create a new WaitSetRunner.
    pub fn new(worker_options: ExecutorWorkerOptions) -> Self {
        let (waitable_sender, waitable_receiver) = unbounded();
        let (task_sender, task_receiver) = unbounded();
        Self {
            wait_set: WaitSet::new(&worker_options.context)
                // SAFETY: This only gets called from Context which ensures that
                // everything is valid when creating a wait set.
                .expect("Unable to create wait set for basic executor"),
            waitable_sender,
            waitable_receiver,
            task_sender,
            task_receiver,
            activity_listeners: Arc::default(),
            guard_condition: worker_options.guard_condition,
            payload: worker_options.payload,
        }
    }

    /// Get the sender that allows users to send new [`Waitable`]s to this
    /// `WaitSetRunner`.
    pub fn waitable_sender(&self) -> UnboundedSender<Waitable> {
        self.waitable_sender.clone()
    }

    /// Get the sender that allows users to send new [`PayloadTask`]s to this
    /// `WaitSetRunner`.
    pub fn payload_task_sender(&self) -> UnboundedSender<PayloadTask> {
        self.task_sender.clone()
    }

    /// Get the group of senders that will be triggered each time the wait set
    /// is woken up. This is used
    pub fn activity_listeners(&self) -> Arc<Mutex<Vec<WeakActivityListener>>> {
        Arc::clone(&self.activity_listeners)
    }

    /// Get the guard condition associated with the wait set of this runner.
    pub fn guard_condition(&self) -> &Arc<GuardCondition> {
        &self.guard_condition
    }

    /// Spawn a thread to run the wait set. You will receive a Promise that will
    /// be resolved once the wait set stops spinning.
    ///
    /// Note that if the user gives a [`SpinOptions::until_promise_resolved`][1],
    /// the best practice is for your executor runtime to swap that out with a
    /// new promise which ensures that the [`ExecutorWorkerOptions::guard_condition`]
    /// will be triggered after the user-provided promise is resolved.
    ///
    /// [1]: crate::SpinOptions::until_promise_resolved
    pub fn run(
        mut self,
        conditions: WaitSetRunConditions,
    ) -> Promise<(Self, Result<(), RclrsError>)> {
        let (sender, promise) = channel();
        std::thread::spawn(move || {
            let result = self.run_blocking(conditions);
            if sender.send((self, result)).is_err() {
                // This is a debug log because this is a normal thing to occur
                // when an executor is winding down.
                log_debug!(
                    "rclrs.wait_set_runner.run",
                    "Unable to return the wait set runner from an async run"
                );
            }
        });

        promise
    }

    /// Run the wait set on the current thread. This will block the execution of
    /// the current thread until the wait set is finished waiting.
    ///
    /// Note that if the user gives a [`SpinOptions::until_promise_resolved`][1],
    /// the best practice is for your executor runtime to swap that out with a
    /// new promise which ensures that the [`ExecutorWorkerOptions::guard_condition`]
    /// will be triggered after the user-provided promise is resolved.
    ///
    /// [1]: crate::SpinOptions::until_promise_resolved
    pub fn run_blocking(&mut self, conditions: WaitSetRunConditions) -> Result<(), RclrsError> {
        let mut first_spin = true;
        let mut listeners = Vec::new();
        loop {
            // TODO(@mxgrey): SmallVec would be better suited here if we are
            // okay with adding that as a dependency.
            let mut new_waitables = Vec::new();
            while let Ok(Some(new_waitable)) = self.waitable_receiver.try_next() {
                new_waitables.push(new_waitable);
            }
            if !new_waitables.is_empty() {
                if let Err(err) = self.wait_set.add(new_waitables) {
                    log_fatal!(
                        "rclrs.wait_set_runner.run_blocking",
                        "Failed to add an item to the wait set: {err}",
                    );
                }
            }

            while let Ok(Some(task)) = self.task_receiver.try_next() {
                task(&mut *self.payload);
            }

            if conditions.only_next_available_work && !first_spin {
                // We've already completed a spin and were asked to only do one,
                // so break here
                return Ok(());
            }
            first_spin = false;

            if conditions.halt_spinning.load(Ordering::Acquire) {
                // The user has manually asked for the spinning to stop
                return Ok(());
            }

            if !conditions.context.ok() {
                // The ROS context has switched to being invalid, so we should
                // stop spinning.
                return Ok(());
            }

            let timeout = conditions.stop_time.map(|t| {
                let timeout = t - Instant::now();
                if timeout < Duration::ZERO {
                    Duration::ZERO
                } else {
                    timeout
                }
            });

            let mut at_least_one = false;
            self.wait_set.wait(timeout, |ready, executable| {
                at_least_one = true;
                // SAFETY: The user of WaitSetRunner is responsible for ensuring
                // the runner has the same payload type as the executables that
                // are given to it.
                unsafe { executable.execute(ready, &mut *self.payload) }
            })?;

            if at_least_one {
                // We drain all listeners from activity_listeners to ensure that we
                // don't get a deadlock from double-locking the activity_listeners
                // mutex while executing one of the listeners. If the listener has
                // access to the Worker<T> then it could attempt to add another
                // listener while we have the vector locked, which would cause a
                // deadlock.
                listeners.extend(
                    self.activity_listeners
                        .lock()
                        .unwrap()
                        .drain(..)
                        .filter_map(|x| x.upgrade()),
                );

                for arc_listener in &listeners {
                    // We pull the callback out of its mutex entirely and release
                    // the lock on the mutex before executing the callback. Otherwise
                    // if the callback triggers its own WorkerActivity to change the
                    // callback then we would get a deadlock from double-locking the
                    // mutex.
                    let listener = { arc_listener.lock().unwrap().take() };
                    if let Some(mut listener) = listener {
                        match &mut listener {
                            ActivityListenerCallback::Listen(listen) => {
                                listen(&mut *self.payload);
                            }
                            ActivityListenerCallback::Inert => {
                                // Do nothing
                            }
                        }

                        // We replace instead of assigning in case the callback
                        // inserted its own
                        arc_listener.lock().unwrap().replace(listener);
                    }
                }

                self.activity_listeners
                    .lock()
                    .unwrap()
                    .extend(listeners.drain(..).map(|x| Arc::downgrade(&x)));
            }

            if let Some(stop_time) = conditions.stop_time {
                if stop_time <= Instant::now() {
                    // If we have exceeded the stop time, then quit spinning.
                    // self.wait_set.wait will not always return Err after a
                    // timeout because it's possible for a primitive to produce
                    // new worker faster than this loop spins.
                    return Err(RclrsError::RclError {
                        code: RclReturnCode::Timeout,
                        msg: None,
                    });
                }
            }
        }
    }
}

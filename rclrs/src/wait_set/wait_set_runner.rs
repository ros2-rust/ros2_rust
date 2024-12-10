use futures::channel::{
    mpsc::{unbounded, UnboundedReceiver, UnboundedSender},
    oneshot::channel,
};

use std::{
    any::Any,
    sync::{atomic::{AtomicBool, Ordering}, Arc},
    time::{Duration, Instant},
};

use crate::{
    Context, Promise, RclrsError, WaitSet, Waitable, GuardCondition, ExecutorWorkerOptions,
};

/// This is a utility class that executors can use to easily run and manage
/// their wait set.
pub struct WaitSetRunner {
    wait_set: WaitSet,
    waitable_sender: UnboundedSender<Waitable>,
    waitable_receiver: UnboundedReceiver<Waitable>,
    guard_condition: Arc<GuardCondition>,
    payload: Box<dyn Any + Send>,
}

/// These are the conditions used by the [`WaitSetRunner`] to determine when it
/// needs to halt.
#[derive(Clone)]
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
    /// Halt trigger that gets set by [`ExecutorCommands`].
    pub halt_spinning: Arc<AtomicBool>,
}

impl WaitSetRunner {
    /// Create a new WaitSetRunner.
    pub fn new(
        worker_options: ExecutorWorkerOptions,
    ) -> Self {
        let (waitable_sender, waitable_receiver) = unbounded();
        Self {
            wait_set: WaitSet::new(&worker_options.context)
                // SAFETY: This only gets called from Context which ensures that
                // everything is valid when creating a wait set.
                .expect("Unable to create wait set for basic executor"),
            waitable_sender,
            waitable_receiver,
            guard_condition: worker_options.guard_condition,
            payload: worker_options.payload,
        }
    }

    /// Get the sender that allows users to send new [`Waitables`] to this
    /// `WaitSetRunner`.
    pub fn sender(&self) -> UnboundedSender<Waitable> {
        self.waitable_sender.clone()
    }

    pub fn guard_condition(&self) -> &Arc<GuardCondition> {
        &self.guard_condition
    }

    /// Spawn a thread to run the wait set. You will receive a Promise that will
    /// be resolved once the wait set stops spinning.
    ///
    /// Note that if the user gives a [`SpinOptions::until_promise_resolved`],
    /// the best practice is for your executor runtime to swap that out with a
    /// new promise which ensures that the [`SpinConditions::guard_condition`]
    /// will be triggered after the user-provided promise is resolved.
    pub fn run(mut self, conditions: WaitSetRunConditions) -> Promise<(Self, Result<(), RclrsError>)> {
        let (sender, promise) = channel();
        std::thread::spawn(move || {
            let result = self.run_blocking(conditions);
            // TODO(@mxgrey): Log any error here when logging becomes available
            sender.send((self, result)).ok();
        });

        promise
    }

    /// Run the wait set on the current thread. This will block the execution of
    /// the current thread until the wait set is finished waiting.
    ///
    /// Note that if the user gives a [`SpinOptions::until_promise_resolved`],
    /// the best practice is for your executor runtime to swap that out with a
    /// new promise which ensures that the [`SpinConditions::guard_condition`]
    /// will be triggered after the user-provided promise is resolved.
    pub fn run_blocking(&mut self, conditions: WaitSetRunConditions) -> Result<(), RclrsError> {
        let mut first_spin = true;
        loop {
            // TODO(@mxgrey): SmallVec would be better suited here if we are
            // okay with adding that as a dependency.
            let mut new_waitables = Vec::new();
            while let Ok(Some(new_waitable)) = self.waitable_receiver.try_next() {
                new_waitables.push(new_waitable);
            }
            if !new_waitables.is_empty() {
                // TODO(@mxgrey): Log any error here when logging becomes available
                self.wait_set.add(new_waitables).ok();
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

            self.wait_set.wait(timeout, |executable| {
                // SAFETY: The user of WaitSetRunner is responsible for ensuring
                // the runner has the same payload type as the executables that
                // are given to it.
                unsafe {
                    executable.execute(&mut self.payload)
                }
            })?;
        }
    }
}

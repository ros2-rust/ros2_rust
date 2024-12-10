use futures::{
    channel::{mpsc::{UnboundedSender, UnboundedReceiver, unbounded}, oneshot},
    future::{BoxFuture, select, select_all, Either},
    task::{waker_ref, ArcWake},
    stream::StreamFuture,
    StreamExt,
};
use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc::{channel, Receiver, Sender},
        Arc, Mutex, Weak,
    },
    task::Context as TaskContext,
    time::Instant,
};

use crate::{
    ExecutorChannel, ExecutorRuntime, SpinConditions, WorkerChannel,
    RclrsError, WaitSetRunner, WaitSetRunConditions, Waitable, log_warn, log_fatal, ToLogParams,
    GuardCondition, ExecutorWorkerOptions,
};

/// The implementation of this runtime is based off of the async Rust reference book:
/// https://rust-lang.github.io/async-book/02_execution/04_executor.html
///
/// This implements a single-threaded async executor. This means the execution of
/// all async tasks will be interlaced on a single thread. This is good for
/// minimizing context switching overhead and preventing the application from
/// consuming more CPU threads than it really needs.
///
/// If you need high-throughput multi-threaded execution, then consider using
/// a different executor.
//
// TODO(@mxgrey): Implement a multi-threaded executor using tokio in a downstream
// crate and refer to it in this documentation.
pub struct BasicExecutorRuntime {
    ready_queue: Receiver<Arc<Task>>,
    task_sender: TaskSender,
    wait_set_runners: Vec<WaitSetRunner>,
    all_guard_conditions: AllGuardConditions,
    new_worker_receiver: Option<StreamFuture<UnboundedReceiver<WaitSetRunner>>>,
    new_worker_sender: UnboundedSender<WaitSetRunner>,
}

#[derive(Clone, Default)]
struct AllGuardConditions {
    inner: Arc<Mutex<Vec<Weak<GuardCondition>>>>,
}

impl AllGuardConditions {
    fn trigger(&self) {
        self.inner.lock().unwrap().retain(|guard_condition| {
            if let Some(guard_condition) = guard_condition.upgrade() {
                guard_condition.trigger();
                true
            } else {
                false
            }
        });
    }

    fn push(&self, guard_condition: Weak<GuardCondition>) {
        self.inner.lock().unwrap().push(guard_condition);
    }
}

impl ExecutorRuntime for BasicExecutorRuntime {
    fn spin(&mut self, conditions: SpinConditions) -> Vec<RclrsError> {
        let conditions = self.process_spin_conditions(conditions);

        let new_workers = self.new_worker_receiver.take().expect(
            "Basic executor was missing its new_worker_receiver at the start of its spinning. \
            This is a critical bug in rclrs. \
            Please report this bug to the maintainers of rclrs by providing a minimum reproduction of the problem."
        );
        let all_guard_conditions = self.all_guard_conditions.clone();

        // futures::channel::oneshot::Receiver is only suitable for async, but
        // we need to block this function from exiting until the WaitSetRunner
        // is returned to self. Therefore we create this blocking channel to
        // prevent the function from returning until the WaitSetRunner has been
        // re-obtained.
        let (worker_sender, worker_receiver) = channel();

        // Use this atomic bool to recognize when we should stop spinning.
        let workers_finished = Arc::new(AtomicBool::new(false));

        // Use this to terminate the spinning once the wait set is finished.
        let workers_finished_clone = Arc::clone(&workers_finished);
        self.task_sender.add_async_task(Box::pin(async move {
            let workers = manage_workers(
                new_workers,
                all_guard_conditions,
                conditions,
            ).await;

            worker_sender.send(workers);
            workers_finished_clone.store(true, Ordering::Release);
        }));

        while let Ok(task) = self.next_task(&workers_finished) {
            // SAFETY: If the mutex is poisoned then we have unrecoverable situation.
            let mut future_slot = task.future.lock().unwrap();
            if let Some(mut future) = future_slot.take() {
                let waker = waker_ref(&task);
                let task_context = &mut TaskContext::from_waker(&waker);
                // Poll the future inside the task so it can do some work and
                // tell us its state.
                if future.as_mut().poll(task_context).is_pending() {
                    // The task is still pending, so return the future to its
                    // task so it can be processed again when it's ready to
                    // continue.
                    *future_slot = Some(future);
                }
            }
        }

        let (runners, new_worker_receiver, errors) = worker_receiver.recv().expect(
            "Basic executor failed to receive the WaitSetRunner at the end of its spinning. \
            This is a critical bug in rclrs. \
            Please report this bug to the maintainers of rclrs by providing a minimum reproduction of the problem."
        );

        self.wait_set_runners = runners;
        self.new_worker_receiver = Some(new_worker_receiver);

        errors
    }

    fn spin_async(
        mut self: Box<Self>,
        conditions: SpinConditions,
    ) -> BoxFuture<'static, (Box<dyn ExecutorRuntime>, Vec<RclrsError>)> {
        let (sender, receiver) = oneshot::channel();
        // Create a thread to run the executor. We should not run the executor
        // as an async task because it blocks its current thread while running.
        // If its future were passed into a different single-threaded async
        // executor then it would block anything else from running on that
        // executor.
        //
        // Theoretically we could design this executor to use async-compatible
        // channels. Then it could run safely inside of a different async
        // executor. But that would probably require us to introduce a new
        // dependency such as tokio.
        std::thread::spawn(move || {
            let result = self.spin(conditions);
            sender.send((self as Box<dyn ExecutorRuntime>, result)).ok();
        });

        Box::pin(async move {
            receiver.await.expect(
                "The basic executor async spin thread was dropped without finishing. \
                This is a critical bug in rclrs. \
                Please report this bug to the maintainers of rclrs by providing a minimum reproduction of the problem."
            )
        })
    }

    fn channel(&self) -> Arc<dyn ExecutorChannel> {
        Arc::new(BasicExecutorChannel {
            task_sender: self.task_sender.clone(),
            new_worker_sender: self.new_worker_sender.clone(),
            all_guard_conditions: self.all_guard_conditions.clone(),
        })
    }
}

impl BasicExecutorRuntime {
    pub(crate) fn new() -> Self {
        let (task_sender, ready_queue) = channel();
        let (new_worker_sender, new_worker_receiver) = unbounded();

        Self {
            ready_queue,
            task_sender: TaskSender { task_sender },
            wait_set_runners: Vec::new(),
            all_guard_conditions: AllGuardConditions::default(),
            new_worker_receiver: Some(new_worker_receiver.into_future()),
            new_worker_sender,
        }
    }

    fn process_spin_conditions(&self, mut conditions: SpinConditions) -> WaitSetRunConditions {
        if let Some(promise) = conditions.options.until_promise_resolved.take() {
            let halt_spinning = Arc::clone(&conditions.halt_spinning);
            let all_guard_conditions = self.all_guard_conditions.clone();
            self.task_sender.add_async_task(Box::pin(async move {
                if let Err(err) = promise.await {
                    // TODO(@mxgrey): We should change this to a log when logging
                    // becomes available.
                    log_warn!(
                        "basic_executor",
                        "Sender for SpinOptions::until_promise_resolved was \
                        dropped, so the Promise will never be fulfilled. \
                        Spinning will stop now. Error message: {err}"
                    );
                }

                // Ordering is very important here. halt_spinning must be set
                // before we lock and trigger the guard conditions. This ensures
                // that when the wait sets wake up, the halt_spinning value is
                // already set to true. Ordering::Release is also important for
                // that purpose.
                //
                // When a new worker is added, the guard conditions will be locked
                // and the new guard condition will be added before checking the
                // value of halt_spinning. That's the opposite order of using
                // these variables. This opposite usage prevents a race condition
                // where the new wait set will start running after we've already
                // triggered all the known guard conditions.
                halt_spinning.store(true, Ordering::Release);
                all_guard_conditions.trigger();
            }));
        }

        WaitSetRunConditions {
            only_next_available_work: conditions.options.only_next_available_work,
            stop_time: conditions.options.timeout.map(|t| Instant::now() + t),
            context: conditions.context,
            halt_spinning: conditions.halt_spinning,
        }
    }

    fn next_task(&mut self, wait_set_finished: &AtomicBool) -> Result<Arc<Task>, ()> {
        if wait_set_finished.load(Ordering::Acquire) {
            // The wait set is done spinning, so we should only pull tasks if
            // they are immediately ready to be performed.
            self.ready_queue.try_recv().map_err(|_| ())
        } else {
            self.ready_queue.recv().map_err(|_| ())
        }
    }
}

struct BasicExecutorChannel {
    task_sender: TaskSender,
    all_guard_conditions: AllGuardConditions,
    new_worker_sender: UnboundedSender<WaitSetRunner>,
}

impl ExecutorChannel for BasicExecutorChannel {
    fn create_worker(
        &self,
        options: ExecutorWorkerOptions,
    ) -> Arc<dyn WorkerChannel> {
        let runner = WaitSetRunner::new(options);
        let waitable_sender = runner.sender();
        self.new_worker_sender.unbounded_send(runner);
        Arc::new(BasicWorkerChannel {
            waitable_sender,
            task_sender: self.task_sender.clone(),
        })
    }

    fn wake_all_wait_sets(&self) {
        self.all_guard_conditions.trigger();
    }
}

struct BasicWorkerChannel {
    task_sender: TaskSender,
    waitable_sender: UnboundedSender<Waitable>,
}

impl WorkerChannel for BasicWorkerChannel {
    fn add_to_waitset(&self, new_entity: Waitable) {
        // TODO(@mxgrey): Log errors here once logging becomes available.
        self.waitable_sender.unbounded_send(new_entity).ok();
    }

    fn add_async_task(&self, f: BoxFuture<'static, ()>) {
        self.task_sender.add_async_task(f);
    }
}

#[derive(Clone)]
struct TaskSender {
    task_sender: Sender<Arc<Task>>,
}

impl TaskSender {
    fn add_async_task(&self, f: BoxFuture<'static, ()>) {
        let task = Arc::new(Task {
            future: Mutex::new(Some(f)),
            task_sender: self.task_sender.clone(),
        });

        // TODO(@mxgrey): Consider logging errors here once logging is available.
        self.task_sender.send(task).ok();
    }
}

struct Task {
    /// This future is held inside an Option because we need to move it in and
    /// out of this `Task` instance without destructuring the `Task` because the
    /// [`ArcWake`] wakeup behavior relies on `Task` having a single instance
    /// that is managed by an Arc.
    ///
    /// We wrap the Option in Mutex because we need to mutate the Option from a
    /// shared borrow that comes from the Arc.
    future: Mutex<Option<BoxFuture<'static, ()>>>,
    task_sender: Sender<Arc<Task>>,
}

/// Implementing this trait gives us a very easy implementation of waking
/// behavior for Task on our BasicExecutorRuntime.
impl ArcWake for Task {
    fn wake_by_ref(arc_self: &Arc<Self>) {
        let cloned = Arc::clone(arc_self);
        arc_self.task_sender.send(cloned).ok();
    }
}

async fn manage_workers(
    mut new_workers: StreamFuture<UnboundedReceiver<WaitSetRunner>>,
    all_guard_conditions: AllGuardConditions,
    conditions: WaitSetRunConditions,
) -> (Vec<WaitSetRunner>, StreamFuture<UnboundedReceiver<WaitSetRunner>>, Vec<RclrsError>) {
    let mut active_runners: Vec<oneshot::Receiver<(WaitSetRunner, Result<(), RclrsError>)>> = Vec::new();
    let mut finished_runners: Vec<WaitSetRunner> = Vec::new();
    let mut errors: Vec<RclrsError> = Vec::new();

    while !active_runners.is_empty() {
        let next_event = select(
            select_all(active_runners),
            new_workers,
        );

        match next_event.await {
            Either::Left((
                (finished_worker, _, remaining_workers),
                new_worker_stream,
            )) => {
                match finished_worker {
                    Ok((runner, result)) => {
                        finished_runners.push(runner);
                        if let Err(err) = result {
                            errors.push(err);
                        }
                    }
                    Err(_) => {
                        log_fatal!(
                            "basic_executor",
                            "WaitSetRunner unexpectedly dropped. This should never happen. \
                            Please report this to the rclrs maintainers with a minimal \
                            reproducible example.",
                        );
                    }
                }

                active_runners = remaining_workers;
                new_workers = new_worker_stream;
            }
            Either::Right((
                (new_worker, new_worker_stream),
                remaining_workers,
            )) => {
                active_runners = remaining_workers.into_inner();

                if let Some(runner) = new_worker {
                    all_guard_conditions.push(Arc::downgrade(runner.guard_condition()));
                    if conditions.halt_spinning.load(Ordering::Acquire) {
                        finished_runners.push(runner);
                    } else {
                        active_runners.push(runner.run(conditions.clone()));
                    }
                }

                new_workers = new_worker_stream.into_future();
            }
        }
    };

    (finished_runners, new_workers, errors)
}

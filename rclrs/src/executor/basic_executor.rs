use futures::{
    future::{BoxFuture, FutureExt},
    task::{waker_ref, ArcWake},
    channel::{oneshot, mpsc::UnboundedSender},
};
use std::{
    future::Future,
    sync::{
        mpsc::{Sender, Receiver, channel},
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    task::Context as TaskContext,
    time::Duration,
};

use crate::{
    executor::{ExecutorRuntime, ExecutorChannel, SpinConditions},
    Waitable, Context, WaitSetRunner,
};

use std::io::Write;

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
    /// We use an Option here because we need to hand the WaitSetRunner off to
    /// another thread while spinning. It should only be None while the spin
    /// function is active. At any other time this should contain Some.
    wait_set_runner: Option<WaitSetRunner>,
}

impl ExecutorRuntime for BasicExecutorRuntime {
    fn spin(&mut self, mut conditions: SpinConditions) {
        self.process_spin_conditions(&mut conditions);

        let wait_set_runner = self.wait_set_runner.take().expect(
            "The wait set runner of the basic executor is missing while beginning to spin. \
            This is a critical bug in rclrs. \
            Please report this bug to the maintainers of rclrs by providing a minimum reproduction of the problem."
        );

        let wait_set_promise = wait_set_runner.run(conditions);
        // futures::channel::oneshot::Receiver is only suitable for async, but
        // we need to block this function from exiting until the WaitSetRunner
        // is returned to self. Therefore we create this blocking channel to
        // prevent the function from returning until the WaitSetRunner has been
        // re-obtained.
        let (wait_set_sender, wait_set_receiver) = channel();

        // Use this atomic bool to recognize when we should stop spinning.
        let wait_set_finished = Arc::new(AtomicBool::new(false));

        // Use this to terminate the spinning once the wait set is finished.
        let wait_set_finished_clone = Arc::clone(&wait_set_finished);
        self.task_sender.add_async_task(Box::pin(async move {
            let wait_set_runner = wait_set_promise.await.expect(
                "The wait set thread of the basic executor dropped prematurely. \
                This is a critical bug in rclrs. \
                Please report this bug to the maintainers of rclrs by providing a minimum reproduction of the problem."
            );
            // TODO(@mxgrey): Log errors here when logging becomes available.
            wait_set_sender.send(wait_set_runner).ok();

            // Notify the main loop that it should stop
            wait_set_finished_clone.store(true, Ordering::Release);
        }));

        let mut count = 0;
        while let Ok(task) = self.next_task(&wait_set_finished) {
            dbg!();
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

            // count += 1;
            // if count > 20 {
            //     panic!("Done {count} iterations");
            // }
        }

        self.wait_set_runner = Some(
            wait_set_receiver.recv().expect(
                "Basic executor failed to receive the WaitSetRunner at the end of its spinning. \
                This is a critical bug in rclrs. \
                Please report this bug to the maintainers of rclrs by providing a minimum reproduction of the problem."
            )
        );
    }

    fn spin_async(
        mut self: Box<Self>,
        conditions: SpinConditions,
    ) -> BoxFuture<'static, Box<dyn ExecutorRuntime>> {
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
            self.spin(conditions);
            sender.send(self as Box<dyn ExecutorRuntime>).ok();
        });

        Box::pin(async move {
            receiver.await.expect(
                "The basic executor async spin thread was dropped without finishing. \
                This is a critical bug in rclrs. \
                Please report this bug to the maintainers of rclrs by providing a minimum reproduction of the problem."
            )
        })
    }

    fn channel(&self) -> Box<dyn ExecutorChannel> {
        let waitable_sender = self.wait_set_runner.as_ref().expect(
            "The wait set runner of the basic executor is missing while creating a channel. \
            This is a critical bug in rclrs. \
            Please report this bug to the maintainers of rclrs by providing a minimum reproduction of the problem."
        )
        .sender();

        Box::new(BasicExecutorChannel {
            task_sender: self.task_sender.clone(),
            waitable_sender,
        })
    }
}

impl BasicExecutorRuntime {
    pub(crate) fn new(context: &Context) -> Self {
        let (task_sender, ready_queue) = channel();
        Self {
            ready_queue,
            task_sender: TaskSender { task_sender },
            wait_set_runner: Some(WaitSetRunner::new(context)),
        }
    }

    fn process_spin_conditions(&self, conditions: &mut SpinConditions) {
        if let Some(promise) = conditions.options.until_promise_resolved.take() {
            let guard_condition = Arc::clone(&conditions.guard_condition);
            let (sender, receiver) = oneshot::channel();
            self.task_sender.add_async_task(Box::pin(async move {
                if let Err(err) = promise.await {
                    // TODO(@mxgrey): We should change this to a log when logging
                    // becomes available.
                    eprintln!(
                        "Sender for SpinOptions::until_promise_resolved was \
                        dropped, so the Promise will never be fulfilled. \
                        Spinning will stop now. Error message: {err}"
                    );
                }
                // TODO(@mxgrey): Log errors here when logging becomes available.
                dbg!();
                std::io::stdout().lock().flush().unwrap();
                guard_condition.trigger().ok();
                sender.send(()).ok();
            }));

            conditions.options.until_promise_resolved = Some(receiver);
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

pub struct BasicExecutorChannel {
    task_sender: TaskSender,
    waitable_sender: UnboundedSender<Waitable>,
}

impl ExecutorChannel for BasicExecutorChannel {
    fn add_async_task(&self, f: BoxFuture<'static, ()>) {
        self.task_sender.add_async_task(f);
    }

    fn add_to_waitset(&self, new_entity: Waitable) {
        // TODO(@mxgrey): Log errors here once logging becomes available.
        self.waitable_sender.unbounded_send(new_entity).ok();
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

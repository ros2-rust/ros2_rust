use futures::{
    future::{BoxFuture, FutureExt},
    task::{waker_ref, ArcWake},
};
use std::{
    future::Future,
    sync::mpsc::{channel, Receiver, Sender},
    sync::{Arc, Mutex},
    task::Context as TaskContext,
    time::Duration,
};

use crate::{
    executor::{ExecutorRuntime, ExecutorChannel, SpinConditions},
    WaitSet, Waitable, Context, WaitSetRunner,
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
    wait_set_runner: WaitSetRunner,
}

impl ExecutorRuntime for BasicExecutorRuntime {
    fn spin(&mut self, mut conditions: SpinConditions) {
        self.process_spin_conditions(&mut conditions);


    }

    fn spin_async(
        self: Box<Self>,
        mut conditions: SpinConditions,
    ) -> BoxFuture<'static, Box<dyn ExecutorRuntime>> {
        self.process_spin_conditions(&mut conditions);

        Box::pin(async move { self as Box<dyn ExecutorRuntime> })
    }

    fn channel(&self) -> Box<dyn ExecutorChannel> {
        Box::new(BasicExecutorChannel {
            task_sender: self.task_sender.clone(),
        })
    }
}

impl BasicExecutorRuntime {
    pub(crate) fn new(context: &Context) -> Self {
        let (task_sender, ready_queue) = channel();

        Self {
            ready_queue,
            task_sender: TaskSender { task_sender },
            wait_set_runner: WaitSetRunner::new(context),
        }
    }

    fn process_spin_conditions(&self, conditions: &mut SpinConditions) {
        if let Some(promise) = conditions.options.until_promise_resolved.take() {
            let guard_condition = Arc::clone(&conditions.guard_condition);
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
                guard_condition.trigger().ok();
            }));
        }
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

pub struct BasicExecutorChannel {
    task_sender: TaskSender,
}

impl ExecutorChannel for BasicExecutorChannel {
    fn add_async_task(&self, f: BoxFuture<'static, ()>) {
        self.task_sender.add_async_task(f);
    }

    fn add_to_waitset(&self, new_entity: Waitable) {

    }
}

struct Task {
    future: Mutex<Option<BoxFuture<'static, ()>>>,
    task_sender: Sender<Arc<Task>>,
}

/// Implementing this trait gives us a very easy implementation of waking
/// behavior for Task on our BasicExecutorRuntime.
impl ArcWake for Task {
    fn wake_by_ref(arc_self: &Arc<Self>) {
        let cloned = Arc::clone(arc_self);
        arc_self.task_sender.send(cloned);
    }
}

mod basic_executor;
pub use self::basic_executor::*;

use crate::{
    rcl_bindings::rcl_context_is_valid,
    Node, NodeOptions, RclReturnCode, RclrsError, WaitSet, Context,
    ContextHandle, Waiter,
};
use std::{
    sync::{Arc, Mutex, Weak},
    time::Duration,
    sync::atomic::{AtomicBool, Ordering},
    future::Future,
};
pub use futures::channel::oneshot::Receiver as Promise;
use futures::{future::{BoxFuture, Either, select}, channel::oneshot};

/// An executor that can be used to create nodes and run their callbacks.
pub struct Executor {
    context: Arc<ContextHandle>,
    commands: Arc<ExecutorCommands>,
    runtime: Box<dyn ExecutorRuntime>,
}

impl Executor {
    /// Access the commands interface for this executor. Use the returned
    /// [`ExecutorCommands`] to create [nodes][Node].
    pub fn commands(&self) -> &Arc<ExecutorCommands> {
        &self.commands
    }

    /// Create a [`Node`] that will run on this Executor.
    pub fn create_node(
        &self,
        options: impl Into<NodeOptions>,
    ) -> Result<Arc<Node>, RclrsError> {
        self.commands.create_node(options)
    }

    /// Spin the Executor. The current thread will be blocked until the Executor
    /// stops spinning.
    ///
    /// [`SpinOptions`] can be used to automatically stop the spinning when
    /// certain conditions are met. Use `SpinOptions::default()` to allow the
    /// Executor to keep spinning indefinitely.
    pub fn spin(&mut self, options: SpinOptions) {
        self.commands.halt.store(false, Ordering::Release);
        let conditions = self.make_spin_conditions(options);
        self.runtime.spin(conditions);
    }

    /// Spin the Executor as an async task. This does not block the current thread.
    /// It also does not prevent your `main` function from exiting while it spins,
    /// so make sure you have a way to keep the application running.
    ///
    /// This will consume the Executor so that the task can run on other threads.
    ///
    /// The async task will run until the [`SpinConditions`] stop the Executor
    /// from spinning. The output of the async task will be the restored Executor,
    /// which you can use to resume spinning after the task is finished.
    pub async fn spin_async(self, options: SpinOptions) -> Self {
        self.commands.halt.store(false, Ordering::Release);
        let conditions = self.make_spin_conditions(options);
        let Self { context, commands, runtime } = self;

        let runtime = runtime.spin_async(conditions).await;
        Self { context, commands, runtime }
    }

    /// Creates a new executor using the provided runtime. Users of rclrs should
    /// use [`Context::create_executor`].
    pub(crate) fn new<E>(context: Arc<ContextHandle>, runtime: E) -> Self
    where
        E: 'static + ExecutorRuntime,
    {
        let commands = Arc::new(ExecutorCommands {
            context: Context { handle: Arc::clone(&context) },
            channel: runtime.channel(),
            halt: Arc::new(AtomicBool::new(false)),
        });

        Self {
            context,
            commands,
            runtime: Box::new(runtime),
        }
    }

    fn make_spin_conditions(&self, options: SpinOptions) -> SpinConditions {
        SpinConditions {
            options,
            halt: Arc::clone(&self.commands.halt),
            context: Context { handle: Arc::clone(&self.context) },
        }
    }
}

/// This allows commands, such as creating a new node, to be run on the executor
/// while the executor is spinning.
pub struct ExecutorCommands {
    context: Context,
    channel: Box<dyn ExecutorChannel>,
    halt: Arc<AtomicBool>,
}

impl ExecutorCommands {
    /// Create a new node that will run on the [`Executor`] that is being commanded.
    pub fn create_node(
        self: &Arc<Self>,
        options: impl Into<NodeOptions>,
    ) -> Result<Arc<Node>, RclrsError> {
        let options: NodeOptions = options.into();
        options.build(self)
    }

    /// Tell the [`Executor`] to halt its spinning.
    pub fn halt(&self) {
        self.halt.store(true, Ordering::Release);
        self.channel.wakeup();
    }

    /// Run a task on the [`Executor`]. If the returned [`Promise`] is dropped
    /// then the task will be dropped, which means it might not run to
    /// completion.
    ///
    /// You can `.await` the output of the promise in an async scope.
    pub fn run<F>(&self, f: F) -> Promise<F::Output>
    where
        F: 'static + Future + Send,
        F::Output: Send,
    {
        let (mut sender, receiver) = oneshot::channel();
        self.channel.add_async_task(Box::pin(
            async move {
                let cancellation = sender.cancellation();
                let output = match select(cancellation, std::pin::pin!(f)).await {
                    // The task was cancelled
                    Either::Left(_) => return,
                    // The task completed
                    Either::Right((output, _)) => output,
                };
                sender.send(output).ok();
            }
        ));

        receiver
    }

    /// Run a task on the [`Executor`]. The task will run to completion even if
    /// you drop the returned [`Promise`].
    ///
    /// You can `.await` the output of the promise in an async scope.
    pub fn run_detached<F>(&self, f: F) -> Promise<F::Output>
    where
        F: 'static + Future + Send,
        F::Output: Send,
    {
        let (sender, receiver) = oneshot::channel();
        self.channel.add_async_task(Box::pin(
            async move {
                sender.send(f.await).ok();
            }
        ));
        receiver
    }

    pub fn context(&self) -> &Context {
        &self.context
    }

    pub(crate) fn add_async_task(&self, f: BoxFuture<'static, ()>) {
        self.channel.add_async_task(f);
    }

    pub(crate) fn add_to_wait_set(&self, waiter: Waiter) {
        self.channel.add_to_waitset(waiter);
    }
}

/// This trait defines the interface for passing new items into an executor to
/// run.
pub trait ExecutorChannel: Send + Sync {
    /// Add a new item for the executor to run.
    fn add_async_task(&self, f: BoxFuture<'static, ()>);

    /// Add new entities to the waitset of the executor.
    fn add_to_waitset(&self, new_entity: Waiter);
}

/// This trait defines the interface for having an executor run.
pub trait ExecutorRuntime {
    /// Get a channel that can add new items for the executor to run.
    fn channel(&self) -> Box<dyn ExecutorChannel>;

    /// Tell the runtime to spin while blocking any further execution until the
    /// spinning is complete.
    fn spin(&mut self, conditions: SpinConditions);

    /// Tell the runtime to spin asynchronously, not blocking the current
    /// thread. The runtime instance will be consumed by this function, but it
    /// must return itself as the output of the [`Future`] that this function
    /// returns.
    fn spin_async(
        self: Box<Self>,
        conditions: SpinConditions,
    ) -> BoxFuture<'static, Box<dyn ExecutorRuntime>>;
}

/// A bundle of optional conditions that a user may want to impose on how long
/// an executor spins for.
///
/// By default the executor will be allowed to spin indefinitely.
#[non_exhaustive]
#[derive(Default)]
pub struct SpinOptions {
    /// Only perform immediately available work. This is similar to spin_once in
    /// rclcpp and rclpy.
    pub only_available_work: bool,
    /// The executor will stop spinning if the future is resolved.
    pub until_future_resolved: Option<BoxFuture<'static, ()>>,
}

/// A bundle of conditions that tell the [`ExecutorRuntime`] how long to keep
/// spinning. This combines conditions that users specify with [`SpinOptions`]
/// and standard conditions that are set by the [`Executor`].
///
/// This struct is for users who are implementing custom executors. Users who
/// are writing applications should use [`SpinOptions`].
#[non_exhaustive]
pub struct SpinConditions {
    /// User-specified optional conditions for spinning.
    pub options: SpinOptions,
    /// Halt trigger that gets set by [`ExecutorCommands`].
    pub halt: Arc<AtomicBool>,
    /// Use this to check [`Context::ok`] to make sure that the context is still
    /// valid. When the context is invalid, the executor runtime should stop
    /// spinning.
    pub context: Context,
}

/// Single-threaded executor implementation.
pub struct SingleThreadedExecutor {
    nodes_mtx: Mutex<Vec<Weak<Node>>>,
}

impl Default for SingleThreadedExecutor {
    fn default() -> Self {
        Self::new()
    }
}

impl SingleThreadedExecutor {
    /// Creates a new executor.
    pub fn new() -> Self {
        SingleThreadedExecutor {
            nodes_mtx: Mutex::new(Vec::new()),
        }
    }

    /// Add a node to the executor.
    pub fn add_node(&self, node: &Arc<Node>) -> Result<(), RclrsError> {
        { self.nodes_mtx.lock().unwrap() }.push(Arc::downgrade(node));
        Ok(())
    }

    /// Remove a node from the executor.
    pub fn remove_node(&self, node: Arc<Node>) -> Result<(), RclrsError> {
        { self.nodes_mtx.lock().unwrap() }
            .retain(|n| !n.upgrade().map(|n| Arc::ptr_eq(&n, &node)).unwrap_or(false));
        Ok(())
    }

    /// Polls the nodes for new messages and executes the corresponding callbacks.
    ///
    /// This function additionally checks that the context is still valid.
    pub fn spin_once(&self, timeout: Option<Duration>) -> Result<(), RclrsError> {
        for node in { self.nodes_mtx.lock().unwrap() }
            .iter()
            .filter_map(Weak::upgrade)
            .filter(|node| unsafe {
                rcl_context_is_valid(&*node.handle.context_handle.rcl_context.lock().unwrap())
            })
        {
            let wait_set = WaitSet::new_for_node(&node)?;
            let ready_entities = wait_set.wait(timeout)?;

            for ready_subscription in ready_entities.subscriptions {
                ready_subscription.execute();
            }

            for ready_client in ready_entities.clients {
                ready_client.execute()?;
            }

            for ready_service in ready_entities.services {
                ready_service.execute()?;
            }
        }

        Ok(())
    }

    /// Convenience function for calling [`SingleThreadedExecutor::spin_once`] in a loop.
    pub fn spin(&self) -> Result<(), RclrsError> {
        while !{ self.nodes_mtx.lock().unwrap() }.is_empty() {
            match self.spin_once(None) {
                Ok(_)
                | Err(RclrsError::RclError {
                    code: RclReturnCode::Timeout,
                    ..
                }) => std::thread::yield_now(),
                error => return error,
            }
        }

        Ok(())
    }
}

mod basic_executor;
pub use self::basic_executor::*;

mod wait_set_runner;
pub use self::wait_set_runner::*;

use crate::{
    Node, NodeOptions, RclrsError, Context, ContextHandle, Waitable, GuardCondition,
};
use std::{
    sync::{Arc, atomic::{AtomicBool, Ordering}},
    time::Duration,
    future::Future,
};
pub use futures::channel::oneshot::Receiver as Promise;
use futures::{
    future::{BoxFuture, Either, select},
    channel::oneshot,
};

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
        let conditions = self.make_spin_conditions(options);
        let Self { context, commands, runtime } = self;

        let runtime = runtime.spin_async(conditions).await;
        Self { context, commands, runtime }
    }

    /// Creates a new executor using the provided runtime. Users of rclrs should
    /// use [`Context::create_executor`].
    pub(crate) fn new<E>(context: Arc<ContextHandle>, runtime: E) -> Self
    where
        E: 'static + ExecutorRuntime + Send,
    {
        let (wakeup_wait_set, waitable) = GuardCondition::new(&context, None);
        let commands = Arc::new(ExecutorCommands {
            context: Context { handle: Arc::clone(&context) },
            channel: runtime.channel(),
            halt_spinning: Arc::new(AtomicBool::new(false)),
            wakeup_wait_set: Arc::new(wakeup_wait_set),
        });

        commands.add_to_wait_set(waitable);

        Self {
            context,
            commands,
            runtime: Box::new(runtime),
        }
    }

    fn make_spin_conditions(&self, options: SpinOptions) -> SpinConditions {
        self.commands.halt_spinning.store(false, Ordering::Release);
        SpinConditions {
            options,
            halt_spinning: Arc::clone(&self.commands.halt_spinning),
            context: Context { handle: Arc::clone(&self.context) },
            guard_condition: Arc::clone(&self.commands.wakeup_wait_set),
        }
    }
}

/// This allows commands, such as creating a new node, to be run on the executor
/// while the executor is spinning.
pub struct ExecutorCommands {
    context: Context,
    channel: Box<dyn ExecutorChannel>,
    halt_spinning: Arc<AtomicBool>,
    wakeup_wait_set: Arc<GuardCondition>,
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
    pub fn halt_spinning(&self) {
        self.halt_spinning.store(true, Ordering::Release);
        // TODO(@mxgrey): Log errors here when logging becomes available
        self.wakeup_wait_set.trigger().ok();
    }

    /// Run a task on the [`Executor`]. If the returned [`Promise`] is dropped
    /// then the task will be dropped, which means it might not run to
    /// completion.
    ///
    /// This differs from [`run`][Self::run] because [`run`][Self::run] will
    /// always run to completion, even if you discard the [`Promise`] that gets
    /// returned. If dropping the [`Promise`] means that you don't need the task
    /// to finish, then this `query` method is what you want.
    ///
    /// You have two ways to obtain the output of the promise:
    /// - `.await` the output of the promise in an async scope
    /// - use [`Promise::try_recv`] to get the output if it is available
    pub fn query<F>(&self, f: F) -> Promise<F::Output>
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
    /// This differs from [`query`][Self::query] because [`query`][Self::query]
    /// will automatically stop running the task if you drop the [`Promise`].
    /// If you want to ensure that the task always runs to completion, then this
    /// `run` method is what you want.
    ///
    /// You can safely discard the promise that is returned to you even if the
    /// compiler gives you a warning about it. Use `let _ = promise;` to suppress
    /// the warning.
    ///
    /// If you choose to keep the promise, you have two ways to obtain its output:
    /// - `.await` the output of the promise in an async scope
    /// - use [`Promise::try_recv`] to get the output if it is available
    pub fn run<F>(&self, f: F) -> Promise<F::Output>
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

    /// Get the context that the executor is associated with.
    pub fn context(&self) -> &Context {
        &self.context
    }

    pub(crate) fn add_to_wait_set(&self, waitable: Waitable) {
        self.channel.add_to_waitset(waitable);
    }

    /// Get a guard condition that can be used to wake up the wait set of the executor.
    pub(crate) fn get_guard_condition(&self) -> &Arc<GuardCondition> {
        &self.wakeup_wait_set
    }
}

/// This trait defines the interface for passing new items into an executor to
/// run.
pub trait ExecutorChannel: Send + Sync {
    /// Add a new item for the executor to run.
    fn add_async_task(&self, f: BoxFuture<'static, ()>);

    /// Add new entities to the waitset of the executor.
    fn add_to_waitset(&self, new_entity: Waitable);
}

/// This trait defines the interface for having an executor run.
pub trait ExecutorRuntime: Send {
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
    /// Only perform the next available work. This is similar to spin_once in
    /// rclcpp and rclpy.
    ///
    /// To only process work that is immediately available without waiting at all,
    /// set a timeout of zero.
    pub only_next_available_work: bool,
    /// The executor will stop spinning if the promise is resolved. The promise
    /// does not need to be fulfilled (i.e. a value was sent), it could also be
    /// cancelled (i.e. the Sender was dropped) and spinning will nevertheless
    /// stop.
    pub until_promise_resolved: Option<Promise<()>>,
    /// Stop waiting after this duration of time has passed. Use `Some(0)` to not
    /// wait any amount of time. Use `None` to wait an infinite amount of time.
    pub timeout: Option<Duration>,
}

impl SpinOptions {
    /// Behave like spin_once in rclcpp and rclpy.
    pub fn spin_once() -> Self {
        Self {
            only_next_available_work: true,
            ..Default::default()
        }
    }

    /// Stop spinning once this promise is resolved.
    pub fn until_promise_resolved(mut self, promise: Promise<()>) -> Self {
        self.until_promise_resolved = Some(promise);
        self
    }

    /// Stop spinning once this durtion of time is reached.
    pub fn timeout(mut self, timeout: Duration) -> Self {
        self.timeout = Some(timeout);
        self
    }
}

/// A bundle of conditions that tell the [`ExecutorRuntime`] how long to keep
/// spinning. This combines conditions that users specify with [`SpinOptions`]
/// and standard conditions that are set by the [`Executor`].
///
/// This struct is only for users who are implementing custom executors. Users
/// who are writing applications should use [`SpinOptions`].
#[non_exhaustive]
pub struct SpinConditions {
    /// User-specified optional conditions for spinning.
    pub options: SpinOptions,
    /// Halt trigger that gets set by [`ExecutorCommands`].
    pub halt_spinning: Arc<AtomicBool>,
    /// Use this to check [`Context::ok`] to make sure that the context is still
    /// valid. When the context is invalid, the executor runtime should stop
    /// spinning.
    pub context: Context,
    /// This is a guard condition which is present in the wait set. The executor
    /// can use this to wake up the wait set.
    pub guard_condition: Arc<GuardCondition>,
}

mod basic_executor;
pub use self::basic_executor::*;

use crate::{
    Context, ContextHandle, GuardCondition, IntoNodeOptions, Node, RclrsError, Waitable,
    WeakActivityListener,
};
pub use futures::channel::oneshot::Receiver as Promise;
use futures::{
    channel::oneshot,
    future::{select, BoxFuture, Either},
};
use std::{
    any::Any,
    future::Future,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::Duration,
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
    pub fn create_node<'a>(
        &'a self,
        options: impl IntoNodeOptions<'a>,
    ) -> Result<Node, RclrsError> {
        let options = options.into_node_options();
        let node = options.build(&self.commands)?;
        Ok(node)
    }

    /// Spin the Executor. The current thread will be blocked until the Executor
    /// stops spinning.
    ///
    /// [`SpinOptions`] can be used to automatically stop the spinning when
    /// certain conditions are met. Use `SpinOptions::default()` to allow the
    /// Executor to keep spinning indefinitely.
    pub fn spin(&mut self, options: SpinOptions) -> Vec<RclrsError> {
        let conditions = self.make_spin_conditions(options);
        let result = self.runtime.spin(conditions);
        self.commands.halt_spinning.store(false, Ordering::Release);
        result
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
    pub async fn spin_async(self, options: SpinOptions) -> (Self, Vec<RclrsError>) {
        let conditions = self.make_spin_conditions(options);
        let Self {
            context,
            commands,
            runtime,
        } = self;

        let (runtime, result) = runtime.spin_async(conditions).await;
        commands.halt_spinning.store(false, Ordering::Release);

        (
            Self {
                context,
                commands,
                runtime,
            },
            result,
        )
    }

    /// Creates a new executor using the provided runtime. Users of rclrs should
    /// use [`Context::create_executor`].
    pub(crate) fn new<E>(context: Arc<ContextHandle>, runtime: E) -> Self
    where
        E: 'static + ExecutorRuntime + Send,
    {
        let executor_channel = runtime.channel();
        let async_worker_commands = ExecutorCommands::impl_create_worker_commands(
            &Context {
                handle: Arc::clone(&context),
            },
            &*executor_channel,
            Box::new(()),
        );

        let commands = Arc::new(ExecutorCommands {
            context: Context {
                handle: Arc::clone(&context),
            },
            executor_channel,
            halt_spinning: Arc::new(AtomicBool::new(false)),
            async_worker_commands,
        });

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
            context: Context {
                handle: Arc::clone(&self.context),
            },
        }
    }
}

/// This allows commands, such as creating a new node, to be run on the executor
/// while the executor is spinning.
pub struct ExecutorCommands {
    context: Context,
    executor_channel: Arc<dyn ExecutorChannel>,
    async_worker_commands: Arc<WorkerCommands>,
    halt_spinning: Arc<AtomicBool>,
}

impl ExecutorCommands {
    /// Create a new node that will run on the [`Executor`] that is being commanded.
    pub fn create_node<'a>(
        self: &Arc<Self>,
        options: impl IntoNodeOptions<'a>,
    ) -> Result<Node, RclrsError> {
        let options = options.into_node_options();
        options.build(self)
    }

    /// Tell the [`Executor`] to halt its spinning.
    pub fn halt_spinning(&self) {
        self.halt_spinning.store(true, Ordering::Release);
        self.executor_channel.wake_all_wait_sets();
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
        self.async_worker_commands
            .channel
            .add_async_task(Box::pin(async move {
                let cancellation = sender.cancellation();
                let output = match select(cancellation, std::pin::pin!(f)).await {
                    // The task was cancelled
                    Either::Left(_) => return,
                    // The task completed
                    Either::Right((output, _)) => output,
                };
                sender.send(output).ok();
            }));

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
        self.async_worker_commands
            .channel
            .add_async_task(Box::pin(async move {
                sender.send(f.await).ok();
            }));
        receiver
    }

    /// Pass in a promise to get a second promise that will notify when the main
    /// promise is fulfilled. This second promise can be passed into
    /// [`SpinOptions::until_promise_resolved`].
    pub fn create_notice<Out>(&self, promise: Promise<Out>) -> (Promise<Out>, Promise<()>)
    where
        Out: 'static + Send,
    {
        let (main_sender, main_receiver) = oneshot::channel();
        let notice_receiver = self.run(async move {
            if let Ok(out) = promise.await {
                main_sender.send(out).ok();
            }
        });

        (main_receiver, notice_receiver)
    }

    /// Get the context that the executor is associated with.
    pub fn context(&self) -> &Context {
        &self.context
    }

    pub(crate) fn add_to_wait_set(&self, waitable: Waitable) {
        self.async_worker_commands.add_to_wait_set(waitable);
    }

    #[cfg(test)]
    pub(crate) fn wake_all_wait_sets(&self) {
        self.executor_channel.wake_all_wait_sets();
    }

    pub(crate) fn async_worker_commands(&self) -> &Arc<WorkerCommands> {
        &self.async_worker_commands
    }

    pub(crate) fn create_worker_commands(
        &self,
        payload: Box<dyn Any + Send>,
    ) -> Arc<WorkerCommands> {
        Self::impl_create_worker_commands(&self.context, &*self.executor_channel, payload)
    }

    /// We separate out this impl function so that we can create the async worker
    /// before the [`ExecutorCommands`] is finished being constructed.
    fn impl_create_worker_commands(
        context: &Context,
        executor_channel: &dyn ExecutorChannel,
        payload: Box<dyn Any + Send>,
    ) -> Arc<WorkerCommands> {
        let (guard_condition, waitable) = GuardCondition::new(&context.handle, None);

        let worker_channel = executor_channel.create_worker(ExecutorWorkerOptions {
            context: context.clone(),
            payload,
            guard_condition: Arc::clone(&guard_condition),
        });

        worker_channel.add_to_wait_set(waitable);

        Arc::new(WorkerCommands {
            channel: worker_channel,
            wakeup_wait_set: guard_condition,
        })
    }
}

/// This is used internally to transmit commands to a specific worker in the
/// executor. It is not accessible to downstream users because it does not
/// retain information about the worker's payload type.
///
/// Downstream users of rclrs should instead use [`Worker`][crate::Worker].
pub(crate) struct WorkerCommands {
    channel: Arc<dyn WorkerChannel>,
    wakeup_wait_set: Arc<GuardCondition>,
}

impl WorkerCommands {
    pub(crate) fn add_to_wait_set(&self, waitable: Waitable) {
        self.channel.add_to_wait_set(waitable);
        let _ = self.wakeup_wait_set.trigger();
    }

    pub(crate) fn run_async<F>(&self, f: F)
    where
        F: 'static + Future<Output = ()> + Send,
    {
        self.channel.add_async_task(Box::pin(f));
    }

    pub(crate) fn run_on_payload(&self, task: PayloadTask) {
        self.channel.send_payload_task(task);
        let _ = self.wakeup_wait_set.trigger();
    }

    pub(crate) fn add_activity_listener(&self, listener: WeakActivityListener) {
        self.channel.add_activity_listener(listener);
    }

    /// Get a guard condition that can be used to wake up the wait set of the executor.
    pub(crate) fn get_guard_condition(&self) -> &Arc<GuardCondition> {
        &self.wakeup_wait_set
    }
}

/// Channel to send commands related to a specific worker.
pub trait WorkerChannel: Send + Sync {
    /// Add a new item for the executor to run.
    fn add_async_task(&self, f: BoxFuture<'static, ()>);

    /// Add new entities to the waitset of the executor.
    fn add_to_wait_set(&self, new_entity: Waitable);

    /// Send a one-time task for the worker to run with its payload.
    fn send_payload_task(&self, f: PayloadTask);

    /// Send something to listen to worker activity.
    fn add_activity_listener(&self, listener: WeakActivityListener);
}

/// Encapsulates a task that can operate on the payload of a worker
pub type PayloadTask = Box<dyn FnOnce(&mut dyn Any) + Send>;

/// This is constructed by [`ExecutorCommands`] and passed to the [`ExecutorRuntime`]
/// to create a new worker. Downstream users of rclrs should not be using this class
/// unless you are implementing your own [`ExecutorRuntime`].
pub struct ExecutorWorkerOptions {
    /// The context that the executor is associated with
    pub context: Context,
    /// The payload that the worker provides to different primitives.
    pub payload: Box<dyn Any + Send>,
    /// The guard condition that should wake up the wait set for the worker.
    pub guard_condition: Arc<GuardCondition>,
}

/// This trait defines the interface for passing new items into an executor to
/// run.
pub trait ExecutorChannel: Send + Sync {
    /// Create a new channel specific to a worker whose payload must be
    /// initialized with the given function.
    fn create_worker(&self, options: ExecutorWorkerOptions) -> Arc<dyn WorkerChannel>;

    /// Wake all the wait sets that are being managed by this executor. This is
    /// used to make sure they respond to [`ExecutorCommands::halt_spinning`].
    fn wake_all_wait_sets(&self);
}

/// This trait defines the interface for having an executor run.
pub trait ExecutorRuntime: Send {
    /// Get a channel that can add new items for the executor to run.
    fn channel(&self) -> Arc<dyn ExecutorChannel>;

    /// Tell the runtime to spin while blocking any further execution until the
    /// spinning is complete.
    fn spin(&mut self, conditions: SpinConditions) -> Vec<RclrsError>;

    /// Tell the runtime to spin asynchronously, not blocking the current
    /// thread. The runtime instance will be consumed by this function, but it
    /// must return itself as the output of the [`Future`] that this function
    /// returns.
    fn spin_async(
        self: Box<Self>,
        conditions: SpinConditions,
    ) -> BoxFuture<'static, (Box<dyn ExecutorRuntime>, Vec<RclrsError>)>;
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
    /// Use default spin options.
    pub fn new() -> Self {
        Self::default()
    }

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

    /// Clone the items inside of [`SpinOptions`] that are able to be cloned.
    /// This will exclude:
    /// - [`until_promise_resolved`][Self::until_promise_resolved]
    pub fn clone_partial(&self) -> SpinOptions {
        SpinOptions {
            only_next_available_work: self.only_next_available_work,
            timeout: self.timeout,
            until_promise_resolved: None,
        }
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
}

impl SpinConditions {
    /// Clone the items inside of [`SpinConditions`] that are able to be cloned.
    /// This will exclude:
    /// - [`until_promise_resolved`][SpinOptions::until_promise_resolved]
    pub fn clone_partial(&self) -> SpinConditions {
        SpinConditions {
            options: self.options.clone_partial(),
            halt_spinning: Arc::clone(&self.halt_spinning),
            context: self.context.clone(),
        }
    }
}

/// This trait allows [`Context`] to create a basic executor.
pub trait CreateBasicExecutor {
    /// Create a basic executor associated with this [`Context`].
    fn create_basic_executor(&self) -> Executor;
}

impl CreateBasicExecutor for Context {
    fn create_basic_executor(&self) -> Executor {
        let runtime = BasicExecutorRuntime::new();
        self.create_executor(runtime)
    }
}

mod basic_executor;
pub use self::basic_executor::*;

use crate::{
    rcl_bindings::rcl_context_is_valid,
    Node, NodeOptions, RclReturnCode, RclrsError, WaitSet, Context,
    ContextHandle,
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

    /// Spin the Executor.
    pub fn spin(&mut self, options: SpinOptions) {
        self.runtime.spin(SpinConditions {
            options,
            halt: Arc::clone(&self.commands.halt),
            context: Context { handle: Arc::clone(&self.context) },
        });
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
}

/// This allows commands, such as creating a new node, to be run on the executor
/// while the executor is spinning.
pub struct ExecutorCommands {
    context: Context,
    channel: Box<dyn ExecutorChannel>,
    halt: Arc<AtomicBool>,
}

impl ExecutorCommands {

    pub fn create_node(
        self: &Arc<Self>,
        options: NodeOptions,
    ) -> Result<Arc<Node>, RclrsError> {
        options.build(self)
    }

    /// Tell the [`Executor`] to halt its spinning.
    pub fn halt(&self) {
        self.halt.store(true, Ordering::Relaxed);
    }

    /// Run a task on the [`Executor`]. If the returned [`Promise`] is dropped
    /// then the task will stop running.
    pub fn run<F>(&self, f: F) -> Promise<F::Output>
    where
        F: 'static + Future + Send + Unpin,
        F::Output: Send,
    {
        let (mut sender, receiver) = oneshot::channel();
        self.channel.add(Box::pin(
            async move {
                let cancellation = sender.cancellation();
                let result = select(cancellation, f).await;
                let output = match result {
                    Either::Left(_) => return,
                    Either::Right((output, _)) => output,
                };
                sender.send(output).ok();
            }
        ));

        receiver
    }

    pub fn context(&self) -> &Context {
        &self.context
    }
}

/// This trait defines the interface for passing new items into an executor to
/// run.
pub trait ExecutorChannel {
    /// Add a new item for the executor to run.
    fn add(&self, f: BoxFuture<'static, ()>);
}

/// This trait defines the interface for having an executor run.
pub trait ExecutorRuntime {
    /// Get a channel that can add new items for the executor to run.
    fn channel(&self) -> Box<dyn ExecutorChannel>;

    /// Tell the executor to spin.
    fn spin(&mut self, conditions: SpinConditions);

    fn async_spin(
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
    /// A limit on how many times the executor should spin before stopping. A
    /// [`None`] value will allow the executor to keep spinning indefinitely.
    pub spin_limit: Option<usize>,
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
    /// Check ok to make sure that the context is still valid. When the context
    /// is invalid, the executor runtime should stop spinning.
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
                ready_subscription.execute()?;
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

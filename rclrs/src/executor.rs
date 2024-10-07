use crate::{
    rcl_bindings::rcl_context_is_valid,
    Node, RclReturnCode, RclrsError, WaitSet, ContextHandle,
};
use std::{
    sync::{Arc, Mutex, Weak},
    time::Duration,
};
pub use futures::channel::oneshot::Receiver as Promise;
use futures::future::BoxFuture;

/// An executor that can be used to run nodes.
pub struct Executor {
    context: Arc<ContextHandle>,
    inner: Box<dyn ExecutorRuntime>,
}

impl Executor {



    /// Creates a new executor using the provided runtime. Users of rclrs should
    /// use [`Context::create_executor`].
    pub(crate) fn new<E>(context: Arc<ContextHandle>, inner: E) -> Self
    where
        E: 'static + ExecutorRuntime,
    {
        Self { context, inner: Box::new(inner) }
    }
}

/// This allows commands, such as creating a new node, to be run on the executor
/// while the executor is spinning.
pub struct ExecutorCommands {
    context: Arc<ContextHandle>,
    channel: Box<dyn ExecutorChannel>,
}

/// This trait defines the interface for passing new items into an executor to
/// run.
pub trait ExecutorChannel {
    /// Add a new item for the executor to run.
    fn add(&self, f: BoxFuture<'static, ()>) -> Promise<()>;
}

/// This trait defines the interface for having an executor run.
pub trait ExecutorRuntime {
    /// Get a channel that can add new items for the executor to run.
    fn channel(&self) -> Arc<dyn ExecutorChannel>;

    /// Tell the executor to spin.
    fn spin(&mut self, conditions: SpinConditions);
}

/// A bundle of conditions that a user may want to impose on how long an
/// executor spins for.
#[non_exhaustive]
pub struct SpinConditions {
    /// A limit on how many times the executor should spin before stopping. A
    /// [`None`] value will allow the executor to keep spinning indefinitely.
    pub spin_limit: Option<usize>,
    /// The executor will stop spinning if the future is resolved.
    pub until_future_resolved: BoxFuture<'static, ()>,
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

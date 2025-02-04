use crate::{
    rcl_bindings::rcl_context_is_valid, Context, ContextHandle, IntoNodeOptions, Node, RclrsError,
    WaitSet,
};
use std::{
    sync::{Arc, Mutex, Weak},
    time::Duration,
};

/// Single-threaded executor implementation.
pub struct Executor {
    context: Arc<ContextHandle>,
    nodes_mtx: Mutex<Vec<Weak<Node>>>,
}

impl Executor {
    /// Create a [`Node`] that will run on this Executor.
    pub fn create_node<'a>(
        &'a self,
        options: impl IntoNodeOptions<'a>,
    ) -> Result<Arc<Node>, RclrsError> {
        let options = options.into_node_options();
        let node = options.build(&self.context)?;
        self.nodes_mtx.lock().unwrap().push(Arc::downgrade(&node));
        Ok(node)
    }

    /// Spin the Executor. The current thread will be blocked until the Executor
    /// stops spinning.
    ///
    /// [`SpinOptions`] can be used to automatically stop the spinning when
    /// certain conditions are met. Use `SpinOptions::default()` to allow the
    /// Executor to keep spinning indefinitely.
    pub fn spin(&mut self, options: SpinOptions) -> Vec<RclrsError> {
        loop {
            if self.nodes_mtx.lock().unwrap().is_empty() {
                // Nothing to spin for, so just quit here
                return Vec::new();
            }

            if let Err(err) = self.spin_once(options.timeout) {
                return vec![err];
            }

            if options.only_next_available_work {
                // We were only suppposed to spin once, so quit here
                return Vec::new();
            }

            std::thread::yield_now();
        }
    }

    /// Polls the nodes for new messages and executes the corresponding callbacks.
    ///
    /// This function additionally checks that the context is still valid.
    fn spin_once(&self, timeout: Option<Duration>) -> Result<(), RclrsError> {
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

        // Clear out any nodes that have been dropped.
        self.nodes_mtx.lock().unwrap().retain(|weak_node| weak_node.strong_count() > 0);

        Ok(())
    }

    /// Used by [`Context`] to create the `Executor`. Users cannot call this
    /// function.
    pub(crate) fn new(context: Arc<ContextHandle>) -> Self {
        Self {
            context,
            nodes_mtx: Mutex::new(Vec::new()),
        }
    }
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

    /// Stop spinning once this durtion of time is reached.
    pub fn timeout(mut self, timeout: Duration) -> Self {
        self.timeout = Some(timeout);
        self
    }
}

/// This trait allows [`Context`] to create a basic executor.
pub trait CreateBasicExecutor {
    /// Create a basic executor associated with this [`Context`].
    fn create_basic_executor(&self) -> Executor;
}

impl CreateBasicExecutor for Context {
    fn create_basic_executor(&self) -> Executor {
        Executor::new(Arc::clone(&self.handle))
    }
}

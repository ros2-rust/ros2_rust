use crate::rcl_bindings::rcl_context_is_valid;
use crate::{Node, RclReturnCode, RclrsError, WaitSet};
use std::sync::{Arc, Mutex, Weak};
use std::time::Duration;

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
            .filter(|node| unsafe { rcl_context_is_valid(&*node.rcl_context_mtx.lock().unwrap()) })
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

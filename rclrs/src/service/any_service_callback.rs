use rosidl_runtime_rs::{Message, Service};

use crate::{WorkerCommands, RclrsError, ServiceHandle, NodeServiceCallback, WorkerServiceCallback};

use std::{any::Any, sync::Arc};

/// An enum capturing the various possible function signatures for service callbacks.
pub enum AnyServiceCallback<T, Payload>
where
    T: Service,
    Payload: 'static + Send,
{
    Node(NodeServiceCallback<T>),
    Worker(WorkerServiceCallback<T, Payload>),
}

impl<T, Payload> AnyServiceCallback<T, Payload>
where
    T: Service,
    Payload: 'static + Send,
{
    pub(super) fn execute(
        &mut self,
        handle: &Arc<ServiceHandle>,
        payload: &mut dyn Any,
        commands: &Arc<WorkerCommands>,
    ) -> Result<(), RclrsError> {
        match self {
            Self::Node(node) => node.execute(Arc::clone(&handle), commands),
            Self::Worker(worker) => worker.execute(handle, payload),
        }
    }
}

impl<T: Service> From<NodeServiceCallback<T>> for AnyServiceCallback<T, ()> {
    fn from(value: NodeServiceCallback<T>) -> Self {
        AnyServiceCallback::Node(value)
    }
}

impl<T: Service, Payload: 'static + Send> From<WorkerServiceCallback<T, Payload>> for AnyServiceCallback<T, Payload> {
    fn from(value: WorkerServiceCallback<T, Payload>) -> Self {
        AnyServiceCallback::Worker(value)
    }
}

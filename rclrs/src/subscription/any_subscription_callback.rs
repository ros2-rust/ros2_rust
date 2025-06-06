use rosidl_runtime_rs::Message;

use crate::{
    subscription::SubscriptionHandle, NodeSubscriptionCallback, RclrsError, WorkerCommands,
    WorkerSubscriptionCallback,
};

use std::{any::Any, sync::Arc};

/// An enum capturing the various possible function signatures for subscription callbacks.
///
/// The correct enum variant is deduced by the [`IntoNodeSubscriptionCallback`][1],
/// [`IntoAsyncSubscriptionCallback`][2], or [`IntoWorkerSubscriptionCallback`][3] trait.
///
/// [1]: crate::IntoNodeSubscriptionCallback
/// [2]: crate::IntoAsyncSubscriptionCallback
/// [3]: crate::IntoWorkerSubscriptionCallback
pub enum AnySubscriptionCallback<T: Message, Payload> {
    /// A callback in the Node scope
    Node(NodeSubscriptionCallback<T>),
    /// A callback in the worker scope
    Worker(WorkerSubscriptionCallback<T, Payload>),
}

impl<T: Message, Payload: 'static> AnySubscriptionCallback<T, Payload> {
    pub(super) fn execute(
        &mut self,
        handle: &Arc<SubscriptionHandle>,
        payload: &mut dyn Any,
        commands: &WorkerCommands,
    ) -> Result<(), RclrsError> {
        match self {
            Self::Node(node) => node.execute(handle, commands),
            Self::Worker(worker) => worker.execute(handle, payload),
        }
    }
}

impl<T: Message> From<NodeSubscriptionCallback<T>> for AnySubscriptionCallback<T, ()> {
    fn from(value: NodeSubscriptionCallback<T>) -> Self {
        AnySubscriptionCallback::Node(value)
    }
}

impl<T: Message, Payload> From<WorkerSubscriptionCallback<T, Payload>>
    for AnySubscriptionCallback<T, Payload>
{
    fn from(value: WorkerSubscriptionCallback<T, Payload>) -> Self {
        AnySubscriptionCallback::Worker(value)
    }
}

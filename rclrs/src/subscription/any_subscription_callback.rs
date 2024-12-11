use rosidl_runtime_rs::Message;

use crate::{
    subscription::SubscriptionHandle, WorkerCommands, RclrsError,
    NodeSubscriptionCallback, WorkerSubscriptionCallback,
};

use std::{any::Any, sync::Arc};

pub enum AnySubscriptionCallback<T: Message, Payload> {
    Node(NodeSubscriptionCallback<T>),
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

impl<T: Message, Payload> From<WorkerSubscriptionCallback<T, Payload>> for AnySubscriptionCallback<T, Payload> {
    fn from(value: WorkerSubscriptionCallback<T, Payload>) -> Self {
        AnySubscriptionCallback::Worker(value)
    }
}

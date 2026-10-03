use rosidl_runtime_rs::Message;

use crate::{NodeSubscriptionCallback, WorkerSubscriptionCallback};

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

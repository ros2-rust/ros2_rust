use crate::{
    Node, AnySubscriptionCallback, IntoNodeSubscriptionCallback,
    IntoAsyncSubscriptionCallback, IntoWorkerSubscriptionCallback, Worker,
};
use std::sync::MutexGuard;
use rosidl_runtime_rs::Message;

pub trait SubscriptionScope<T: Message> {
    type SetCallback<'a>;
    type Payload;

    fn set_callback<'a>(
        guard: MutexGuard<'a, AnySubscriptionCallback<T, Self::Payload>>,
    ) -> Self::SetCallback<'a>;
}

pub struct SetNodeSubscriptionCallback<'a, T: Message> {
    guard: MutexGuard<'a, AnySubscriptionCallback<T, ()>>,
}

impl<'a, T: Message> SetNodeSubscriptionCallback<'a, T> {
    /// Set the callback of this subscription, replacing the callback that was
    /// previously set.
    ///
    /// This can be used even if the subscription previously used an async callback.
    pub fn set_regular<Args>(&mut self, callback: impl IntoNodeSubscriptionCallback<T, Args>) {
        let callback = callback.into_node_subscription_callback();
        *self.guard = callback;
    }

    /// Set the callback of this subscription, replacing the callback that was
    /// previously set.
    ///
    /// This can be used even if the subscription previously used a non-async callback.
    pub fn set_async<Args>(&mut self, callback: impl IntoAsyncSubscriptionCallback<T, Args>) {
        let callback = callback.into_async_subscription_callback();
        *self.guard = callback;
    }
}

impl<T: Message> SubscriptionScope<T> for Node {
    type SetCallback<'a> = SetNodeSubscriptionCallback<'a, T>;
    type Payload = ();

    fn set_callback<'a>(
        guard: MutexGuard<'a, AnySubscriptionCallback<T, Self::Payload>>,
    ) -> Self::SetCallback<'a> {
        SetNodeSubscriptionCallback { guard }
    }
}

pub struct SetWorkerSubscriptionCallback<'a, T: Message, Payload> {
    guard: MutexGuard<'a, AnySubscriptionCallback<T, Payload>>,
}

impl<'a, T: Message, Payload: 'static> SetWorkerSubscriptionCallback<'a, T, Payload> {
    pub fn set<Args>(&mut self, callback: impl IntoWorkerSubscriptionCallback<T, Payload, Args>) {
        let callback = callback.into_worker_subscription_callback();
        *self.guard = callback;
    }
}

impl<T: Message, Payload: 'static> SubscriptionScope<T> for Worker<Payload> {
    type SetCallback<'a> = SetWorkerSubscriptionCallback<'a, T, Payload>;
    type Payload = Payload;

    fn set_callback<'a>(
        guard: MutexGuard<'a, AnySubscriptionCallback<T, Self::Payload>>,
    ) -> Self::SetCallback<'a> {
        SetWorkerSubscriptionCallback { guard }
    }
}

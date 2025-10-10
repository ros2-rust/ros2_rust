use super::GoalClientLifecycle;
use crate::GoalStatusCode;
use futures::{future::Shared, FutureExt};
use rosidl_runtime_rs::Action;
use std::{
    future::Future,
    ops::{Deref, DerefMut},
    pin::Pin,
    sync::Arc,
    task::{Context, Poll},
};
use tokio::sync::oneshot::Receiver;

/// This struct allows you to receive the result of the goal. Through the [`DerefMut`]
/// trait you can use the [`oneshot::Receiver`][Receiver] API to await the final
/// result. Note that it can only be received once.
pub struct ResultClient<A: Action> {
    receiver: Shared<Receiver<(GoalStatusCode, A::Result)>>,
    /// This keeps track of whether any goal client components are still being used.
    /// This must come after the receiver in the struct to ensure cleanup is done
    /// correctly.
    #[allow(unused)]
    lifecycle: Arc<GoalClientLifecycle<A>>,
}

impl<A: Action> Clone for ResultClient<A> {
    fn clone(&self) -> Self {
        Self {
            receiver: self.receiver.clone(),
            lifecycle: Arc::clone(&self.lifecycle),
        }
    }
}

impl<A: Action> ResultClient<A> {
    pub(super) fn new(
        receiver: Receiver<(GoalStatusCode, A::Result)>,
        lifecycle: GoalClientLifecycle<A>,
    ) -> Self {
        Self {
            receiver: receiver.shared(),
            lifecycle: Arc::new(lifecycle),
        }
    }
}

impl<A: Action> Deref for ResultClient<A> {
    type Target = Shared<Receiver<(GoalStatusCode, A::Result)>>;
    fn deref(&self) -> &Self::Target {
        &self.receiver
    }
}

impl<A: Action> DerefMut for ResultClient<A> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.receiver
    }
}

impl<A: Action> Future for ResultClient<A> {
    type Output = (GoalStatusCode, A::Result);

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        Future::poll(Pin::new(&mut self.get_mut().receiver), cx)
            // SAFETY: The Receiver returns an Err if the sender is dropped, but
            // the ResultClient makes sure that the sender is alive in
            // the ActionClient, so we can always safely unwrap this.
            .map(|result| result.unwrap())
    }
}

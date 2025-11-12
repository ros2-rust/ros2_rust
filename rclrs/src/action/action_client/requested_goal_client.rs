use super::GoalClientLifecycle;
use crate::{GoalClient, GoalUuid};
use rosidl_runtime_rs::Action;
use std::{
    future::Future,
    ops::{Deref, DerefMut},
    pin::Pin,
    task::{Context, Poll},
};
use tokio::sync::oneshot::Receiver;

/// This struct allows you to receive a [`GoalClient`] for a goal that you
/// requested. Call `.await` on this to obtain the response to the goal request.
/// Note that the [`GoalClient`] can only be received once.
///
/// Through the [`DerefMut`] trait you can use the [`oneshot::Receiver`][Receiver] API.
///
/// If the action server rejects the goal then this will yield a [`None`] instead
/// of a [`GoalClient`].
pub struct RequestedGoalClient<A: Action> {
    goal_id: GoalUuid,
    receiver: Receiver<Option<GoalClient<A>>>,
    /// This keeps track of whether any goal client components are still being used.
    /// This must come after the receiver in the struct to ensure cleanup is done
    /// correctly.
    #[allow(unused)]
    lifecycle: GoalClientLifecycle<A>,
}

impl<A: Action> RequestedGoalClient<A> {
    /// Get the unique ID for the goal that has been requested.
    pub fn goal_id(&self) -> &GoalUuid {
        &self.goal_id
    }

    pub(super) fn new(
        goal_id: GoalUuid,
        receiver: Receiver<Option<GoalClient<A>>>,
        lifecycle: GoalClientLifecycle<A>,
    ) -> Self {
        Self {
            goal_id,
            receiver,
            lifecycle,
        }
    }
}

impl<A: Action> Deref for RequestedGoalClient<A> {
    type Target = Receiver<Option<GoalClient<A>>>;
    fn deref(&self) -> &Self::Target {
        &self.receiver
    }
}

impl<A: Action> DerefMut for RequestedGoalClient<A> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.receiver
    }
}

impl<A: Action> Future for RequestedGoalClient<A> {
    type Output = Option<GoalClient<A>>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        Future::poll(Pin::new(&mut self.get_mut().receiver), cx)
            // SAFETY: The Receiver returns an Err if the sender is dropped, but
            // the RequestedGoalClient makes sure that the sender is alive in
            // the ActionClient, so we can always safely unwrap this.
            .map(|result| result.unwrap())
    }
}

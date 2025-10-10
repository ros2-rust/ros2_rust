use super::GoalClientLifecycle;
use crate::{ActionClient, CancelResponse, GoalUuid, MultiCancelResponse, RclrsError};
use rosidl_runtime_rs::Action;
use std::{
    future::Future,
    ops::{Deref, DerefMut},
    pin::Pin,
    sync::Arc,
    task::{Context, Poll},
};
use tokio::sync::oneshot::Receiver;

/// This can be used to request the cancellation of a specific goal. When you
/// put in the request you will get a one-shot receiver which will tell you
/// whether the request was accepted or rejected.
pub struct CancellationClient<A: Action> {
    client: ActionClient<A>,
    goal_id: GoalUuid,
}

impl<A: Action> Clone for CancellationClient<A> {
    fn clone(&self) -> Self {
        Self {
            client: Arc::clone(&self.client),
            goal_id: self.goal_id,
        }
    }
}

impl<A: Action> CancellationClient<A> {
    /// Ask the goal to cancel. You will receiver a [`CancelResponseClient`]
    /// which you can use to await the response from the action server.
    ///
    /// If an rcl error occurs, this will panic. This function is not likely to
    /// result in any rcl errors.
    pub fn cancel(&self) -> CancelResponseClient<A> {
        self.try_cancel().unwrap()
    }

    /// A version of [`Self::cancel`] which does not panic when an error occurs.
    pub fn try_cancel(&self) -> Result<CancelResponseClient<A>, RclrsError> {
        self.client
            .board
            .request_single_cancel(&self.client, self.goal_id)
    }

    pub(super) fn new(client: ActionClient<A>, goal_id: GoalUuid) -> Self {
        Self { client, goal_id }
    }
}

/// This will allow you to receive a response to your cancellation request so you
/// can know if the cancellation was successful or not.
///
/// You can obtain one of these using [`CancellationClient`] (which is provided
/// by [`GoalClient`][1]) or using [`ActionClientState::cancel_goal`][2].
///
/// [1]: crate::GoalClient
/// [2]: crate::ActionClientState::cancel_goal
pub struct CancelResponseClient<A: Action> {
    receiver: Receiver<CancelResponse>,
    /// This keeps track of whether any goal client components are still being used.
    /// This must come after the receiver in the struct to ensure cleanup is done
    /// correctly.
    #[allow(unused)]
    lifecycle: GoalClientLifecycle<A>,
}

impl<A: Action> Deref for CancelResponseClient<A> {
    type Target = Receiver<CancelResponse>;
    fn deref(&self) -> &Self::Target {
        &self.receiver
    }
}

impl<A: Action> DerefMut for CancelResponseClient<A> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.receiver
    }
}

impl<A: Action> CancelResponseClient<A> {
    pub(super) fn new(
        receiver: Receiver<CancelResponse>,
        lifecycle: GoalClientLifecycle<A>,
    ) -> Self {
        Self {
            receiver,
            lifecycle,
        }
    }
}

impl<A: Action> Future for CancelResponseClient<A> {
    type Output = CancelResponse;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        Future::poll(Pin::new(&mut self.get_mut().receiver), cx)
            // SAFETY: The Receiver returns an Err if the sender is dropped, but
            // the CancelResponseClient makes sure that the sender is alive in
            // the ActionClient, so we can always safely unwrap this.
            .map(|result| result.unwrap())
    }
}

/// This will allow you to receive a response to a cancellation request that
/// impacts multiple goals.
///
/// You can obtain one of these using [`ActionClientState::cancel_all_goals`][1]
/// or [`ActionClientState::cancel_goals_prior_to`][2].
///
/// [1]: crate::ActionClientState::cancel_all_goals
/// [2]: crate::ActionClientState::cancel_goals_prior_to
pub struct MultiCancelResponseClient<A: Action> {
    receiver: Receiver<MultiCancelResponse>,
    /// This keeps track of whether any goal client components are still being used.
    /// This must come after the receiver in the struct to ensure cleanup is done
    /// correctly.
    #[allow(unused)]
    lifecycle: GoalClientLifecycle<A>,
}

impl<A: Action> Deref for MultiCancelResponseClient<A> {
    type Target = Receiver<MultiCancelResponse>;
    fn deref(&self) -> &Self::Target {
        &self.receiver
    }
}

impl<A: Action> DerefMut for MultiCancelResponseClient<A> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.receiver
    }
}

impl<A: Action> MultiCancelResponseClient<A> {
    pub(super) fn new(
        receiver: Receiver<MultiCancelResponse>,
        lifecycle: GoalClientLifecycle<A>,
    ) -> Self {
        Self {
            receiver,
            lifecycle,
        }
    }
}

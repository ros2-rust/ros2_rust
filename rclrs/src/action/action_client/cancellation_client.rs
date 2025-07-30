use crate::{ActionClient, CancelResponseCode, GoalUuid, RclrsError};
use super::GoalClientLifecycle;
use tokio::sync::oneshot::Receiver;
use rosidl_runtime_rs::Action;
use std::{
    sync::Arc,
    ops::{Deref, DerefMut}
};

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
        self.client.board.request_single_cancel(Arc::clone(&self.client), self.goal_id)
    }

    pub(super) fn new(
        client: ActionClient<A>,
        goal_id: GoalUuid,
    ) -> Self {
        Self { client, goal_id }
    }
}

/// This will allow you to receive a response to your cancellation request so you
/// can know if the cancellation was successful or not.
pub struct CancelResponseClient<A: Action> {
    receiver: Receiver<CancelResponseCode>,
    /// This keeps track of whether any goal client components are still being used.
    /// This must come after the receiver in the struct to ensure cleanup is done
    /// correctly.
    #[allow(unused)]
    lifecycle: GoalClientLifecycle<A>,
}

impl<A: Action> Deref for CancelResponseClient<A> {
    type Target = Receiver<CancelResponseCode>;
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
        receiver: Receiver<CancelResponseCode>,
        lifecycle: GoalClientLifecycle<A>,
    ) -> Self {
        Self { receiver, lifecycle }
    }
}

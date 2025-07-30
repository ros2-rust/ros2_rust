use crate::GoalClient;
use super::GoalClientLifecycle;
use rosidl_runtime_rs::Action;
use tokio::sync::oneshot::Receiver;
use std::ops::{Deref, DerefMut};

/// This struct allows you to receive a [`GoalClient`] for a goal that you
/// requested. Through the [`DerefMut`] trait you can use the [`oneshot::Receiver`][Receiver]
/// API to await the goal client. Note that it can only be received once.
///
/// If the action server rejects the goal then this will yield a [`None`] instead
/// of a [`GoalClient`].
pub struct RequestedGoalClient<A: Action> {
    receiver: Receiver<Option<GoalClient<A>>>,
    /// This keeps track of whether any goal client components are still being used.
    /// This must come after the receiver in the struct to ensure cleanup is done
    /// correctly.
    #[allow(unused)]
    lifecycle: GoalClientLifecycle<A>,
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

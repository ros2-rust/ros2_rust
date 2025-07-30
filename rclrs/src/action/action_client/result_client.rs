use crate::GoalStatusCode;
use super::GoalClientLifecycle;
use rosidl_runtime_rs::Action;
use tokio::sync::oneshot::Receiver;
use std::ops::{Deref, DerefMut};

/// This struct allows you to receive the result of the goal. Through the [`DerefMut`]
/// trait you can use the [`oneshot::Receiver`][Receiver] API to await the final
/// result. Note that it can only be received once.
pub struct ResultClient<A: Action> {
    receiver: Receiver<(GoalStatusCode, A::Result)>,
    /// This keeps track of whether any goal client components are still being used.
    /// This must come after the receiver in the struct to ensure cleanup is done
    /// correctly.
    #[allow(unused)]
    lifecycle: GoalClientLifecycle<A>,
}

impl<A: Action> Deref for ResultClient<A> {
    type Target = Receiver<(GoalStatusCode, A::Result)>;
    fn deref(&self) -> &Self::Target {
        &self.receiver
    }
}

impl<A: Action> DerefMut for ResultClient<A> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.receiver
    }
}


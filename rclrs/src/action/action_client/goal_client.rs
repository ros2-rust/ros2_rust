use crate::{FeedbackClient, StatusClient};
use super::{GoalClientLifecycle};
use rosidl_runtime_rs::Action;
use tokio::sync::{
    oneshot::Receiver,
};
use std::{
    ops::{Deref, DerefMut},
    sync::Arc,
};

/// The goal client bundles a set of receivers that will allow you to await
/// different information from the action server, such as feedback messages,
/// status updates, and the final result. It also provides a way to request
/// cancelling a goal.
///
/// This struct is designed to be [destructured][1] so each of its fields can be
/// used independently. This is important because many of them require mutable
/// access (or one-time access) to be used, so being blocked behind `&mut self`
/// would make it impossible to use them independently, e.g. to await more than
/// one at a time.
///
/// [1]: https://doc.rust-lang.org/rust-by-example/flow_control/match/destructuring/destructure_structures.html
pub struct GoalClient<A: Action> {
    /// Receive feedback messages for the goal.
    pub feedback: FeedbackClient<A>,
    /// Watch the status of the goal.
    pub status: StatusClient<A>,
    /// Get the final result of the goal.
    pub result: Receiver<A::Result>
}

/// This struct allows you to receive the result of the goal. Through the [`DerefMut`]
/// trait you can use the [`oneshot::Receiver`][Receiver] API to await the final
/// result. Note that it can only be received once.
pub struct ResultClient<A: Action> {
    receiver: Receiver<A::Result>,
    /// This keeps track of whether any goal client components are still being used.
    /// This must come after the receiver in the struct to ensure cleanup is done
    /// correctly.
    #[allow(unused)]
    lifecycle: Arc<GoalClientLifecycle<A>>,
}

impl<A: Action> Deref for ResultClient<A> {
    type Target = Receiver<A::Result>;
    fn deref(&self) -> &Self::Target {
        &self.receiver
    }
}

impl<A: Action> DerefMut for ResultClient<A> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.receiver
    }
}


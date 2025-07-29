use crate::{GoalStatus, GoalUuid};
use super::{ActionClientGoalBoard, GoalClientLifecycle};
use rosidl_runtime_rs::Action;
use tokio::sync::{
    watch::Receiver as Watcher,
    mpsc::UnboundedReceiver,
    oneshot::Receiver,
};
use std::{
    ops::{Deref, DerefMut},
    sync::Arc,
};

/// This struct allows you to receive feedback messages for the goal. Through the
/// [`DerefMut`] trait you can use the [`UnboundedReceiver`] API to await new
/// feedback messages.
pub struct FeedbackClient<A: Action> {
    receiver: UnboundedReceiver<A::Feedback>,
    /// This keeps track of whether any goal client components are still being used.
    /// This must come after the receiver in the struct to ensure cleanup is done
    /// correctly.
    #[allow(unused)]
    lifecycle: Arc<GoalClientLifecycle<A>>,
}

impl<A: Action> Deref for FeedbackClient<A> {
    type Target = UnboundedReceiver<A::Feedback>;
    fn deref(&self) -> &Self::Target {
        &self.receiver
    }
}

impl<A: Action> DerefMut for FeedbackClient<A> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.receiver
    }
}

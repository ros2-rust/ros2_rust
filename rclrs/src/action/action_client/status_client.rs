use crate::{GoalStatusCode, GoalUuid, FeedbackClient};
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

/// This struct allows you to monitor the status of the goal. Through the
/// [`DerefMut`] trait you can use the [`watch::Receiver`][Watcher] API to
/// check the latest status and monitor changes.
pub struct StatusClient<A: Action> {
    receiver: Watcher<GoalStatusCode>,
    /// This keeps track of whether any goal client components are still being used.
    /// This must come after the receiver in the struct to ensure cleanup is done
    /// correctly.
    #[allow(unused)]
    lifecycle: Arc<GoalClientLifecycle<A>>,
}

impl<A: Action> Clone for StatusClient<A> {
    fn clone(&self) -> Self {
        Self {
            receiver: self.receiver.clone(),
            lifecycle: Arc::clone(&self.lifecycle),
        }
    }
}

impl<A: Action> Deref for StatusClient<A> {
    type Target = Watcher<GoalStatusCode>;
    fn deref(&self) -> &Self::Target {
        &self.receiver
    }
}

impl<A: Action> DerefMut for StatusClient<A> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.receiver
    }
}

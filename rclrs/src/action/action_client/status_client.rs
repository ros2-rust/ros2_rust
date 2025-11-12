use super::GoalClientLifecycle;
use crate::{GoalStatus, GoalUuid};
use rosidl_runtime_rs::Action;
use std::{
    ops::{Deref, DerefMut},
    sync::Arc,
};
use tokio::sync::{mpsc::UnboundedReceiver, watch::Receiver as Watcher};

/// This struct allows you to receive every status update experienced by a goal.
///
/// If you only care about checking the latest status, you can call [`Self::watch`]
/// to use a [`StatusWatcher`] instead.
pub struct StatusClient<A: Action> {
    receiver: UnboundedReceiver<GoalStatus>,
    goal_id: GoalUuid,
    #[allow(unused)]
    lifecycle: GoalClientLifecycle<A>,
}

impl<A: Action> Clone for StatusClient<A> {
    fn clone(&self) -> Self {
        self.lifecycle.client.receive_status(self.goal_id)
    }
}

impl<A: Action> Deref for StatusClient<A> {
    type Target = UnboundedReceiver<GoalStatus>;
    fn deref(&self) -> &Self::Target {
        &self.receiver
    }
}

impl<A: Action> DerefMut for StatusClient<A> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.receiver
    }
}

impl<A: Action> StatusClient<A> {
    /// Watch the latest value of the goal status.
    pub fn watch(&self) -> StatusWatcher<A> {
        self.lifecycle.client.watch_status(self.goal_id)
    }

    pub(super) fn new(
        receiver: UnboundedReceiver<GoalStatus>,
        goal_id: GoalUuid,
        lifecycle: GoalClientLifecycle<A>,
    ) -> Self {
        Self {
            receiver,
            goal_id,
            lifecycle,
        }
    }
}

/// This struct allows you to monitor the status of the goal. Through the
/// [`DerefMut`] trait you can use the [`watch::Receiver`][Watcher] API to
/// check the latest status and monitor changes.
pub struct StatusWatcher<A: Action> {
    watcher: Watcher<GoalStatus>,
    /// This keeps track of whether any goal client components are still being used.
    /// This must come after the receiver in the struct to ensure cleanup is done
    /// correctly.
    #[allow(unused)]
    lifecycle: Arc<GoalClientLifecycle<A>>,
}

impl<A: Action> Clone for StatusWatcher<A> {
    fn clone(&self) -> Self {
        Self {
            watcher: self.watcher.clone(),
            lifecycle: Arc::clone(&self.lifecycle),
        }
    }
}

impl<A: Action> Deref for StatusWatcher<A> {
    type Target = Watcher<GoalStatus>;
    fn deref(&self) -> &Self::Target {
        &self.watcher
    }
}

impl<A: Action> DerefMut for StatusWatcher<A> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.watcher
    }
}

impl<A: Action> StatusWatcher<A> {
    pub(super) fn new(
        watcher: Watcher<GoalStatus>,
        lifecycle: Arc<GoalClientLifecycle<A>>,
    ) -> Self {
        Self { watcher, lifecycle }
    }
}

use crate::{
    ActionServer, ActionServerState, IntoActionServerOptions, Node, RclrsError, RequestedGoal,
    TerminatedGoal,
};
use rosidl_runtime_rs::Action;
use std::{
    future::Future,
    ops::{Deref, DerefMut},
    sync::Arc,
};
use tokio::sync::mpsc::{unbounded_channel, UnboundedReceiver};

/// This is an alternative tool for implementing an [`ActionServer`]. Instead of
/// specifying a callback to receive goal requests, you can use this receiver to
/// obtain incoming goal requests and then process them in an async task.
///
/// The [`ActionGoalReceiver`] may have some advantages over the [`ActionServer`]
/// for action servers that support multiple simultaneous goals which may interact
/// with each other and therefore need to be processed within the same async task.
//
// TODO(@mxgrey): Add usage examples here.
pub struct ActionGoalReceiver<A: Action> {
    server: ActionServerState<A>,
    receiver: UnboundedReceiver<RequestedGoal<A>>,
}

impl<A: Action> Deref for ActionGoalReceiver<A> {
    type Target = UnboundedReceiver<RequestedGoal<A>>;

    fn deref(&self) -> &Self::Target {
        &self.receiver
    }
}

impl<A: Action> DerefMut for ActionGoalReceiver<A> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.receiver
    }
}

impl<A: Action> ActionGoalReceiver<A> {
    /// Change this action goal receiver into a callback-based action server.
    ///
    /// It is unusual to switch from an action goal receiver to an action server,
    /// so consider carefully whether this is what you really want to do. Usually
    /// an action server is created by [`NodeState::create_action_server`].
    #[must_use]
    pub fn into_action_server<Task>(
        self,
        callback: impl FnMut(RequestedGoal<A>) -> Task + Send + Sync + 'static,
    ) -> ActionServer<A>
    where
        Task: Future<Output = TerminatedGoal> + Send + Sync + 'static,
    {
        let Self { server, receiver } = self;
        server.drain_receiver_into_callback(receiver, callback);
        Arc::new(server)
    }

    pub(crate) fn new<'a>(
        node: &Node,
        options: impl IntoActionServerOptions<'a>,
    ) -> Result<Self, RclrsError> {
        let (sender, receiver) = unbounded_channel();
        let server = ActionServerState::new_for_receiver(node, options, sender)?;
        Ok(Self { server, receiver })
    }

    pub(super) fn from_server(server: ActionServerState<A>) -> Self {
        let (sender, receiver) = unbounded_channel();
        server.set_goal_sender(sender);
        Self { server, receiver }
    }
}

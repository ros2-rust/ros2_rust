use super::{CancellingGoal, ExecutingGoal, LiveActionServerGoal};
use std::{
    future::Future,
    sync::Arc
};
use rosidl_runtime_rs::ActionImpl;

/// This manages a goal which has been accepted but has not begun executing yet.
/// It is allowed to transition into the executing or cancelling state.
pub struct AcceptedGoal<A: ActionImpl> {
    live: Arc<LiveActionServerGoal<A>>,
}

impl<A: ActionImpl> AcceptedGoal<A> {
    /// Get the goal of this action.
    pub fn goal(&self) -> &Arc<A::Goal> {
        self.live.goal()
    }

    /// Transition the goal into the executing state. This will allow you to
    /// start sending feedback and transition to other states such as cancelling,
    /// succeedded, and aborted.
    ///
    /// You can use this even if an action client has requested a cancellation,
    /// but the cancellation request will be ignored until you handle it with
    /// [`ExecutingGoal`].
    #[must_use]
    pub fn execute(self) -> ExecutingGoal<A> {
        ExecutingGoal::new(self.live)
    }

    /// Process a [`Future`] until it is finished or until a cancellation request
    /// is received.
    ///
    /// If the [`Future`] finishes, its output will be provided in [`Ok`]. If a
    /// cancellation request is received before the [`Future`] is finished, you
    /// will receive an [`Err`] with the current state of the [`Future`], which
    /// you can continue processing later if you choose.
    ///
    /// After the cancellation request is received, you will still need to trigger
    /// [`Self::begin_cancelling`] or [`Self::reject_cancellation`] to respond to
    /// the request. Otherwise the cancellation request will not receive a response
    /// until the goal reaches a terminal state.
    //
    // TODO(@mxgrey): Add a doctest and example for this.
    pub async fn until_cancel_requested<F: Future + Unpin>(&self, f: F) -> Result<F::Output, F> {
        self.live.cancellation().until_cancel_requested(f).await
    }

    /// Transition the goal into the cancelling state.
    ///
    /// This does not require an action client to request a cancellation. Instead
    /// you may act as the action client and commanding that the goal transition
    /// into cancelling.
    ///
    /// If there are any open cancellation requests for this goal from any action
    /// clients, they will all be notified that the cancellation is accepted.
    ///
    /// For the goal to reach the cancelled state, you must follow this up with
    /// [`CancellingGoal::cancelled_with`].
    #[must_use]
    pub fn begin_cancelling(self) -> CancellingGoal<A> {
        CancellingGoal::new(self.live)
    }

    /// If there are any open cancellation requests for this goal, reject them.
    /// This does not transition the goal in any way.
    pub fn reject_cancellation(&self) {
        self.live.reject_cancellation();
    }

    /// Transition the goal into either the executing mode or the cancelling mode.
    ///
    /// If any action client has sent a cancellation request, this will automatically
    /// transition the goal into the cancelling state and provide you with a
    /// [`CancellingGoal`]. Otherwise the goal will transition into the executing
    /// state and you will receive an [`ExecutingGoal`].
    #[must_use]
    pub fn begin(self) -> BeginAcceptedGoal<A> {
        if self.live.cancel_requested() {
            BeginAcceptedGoal::Execute(self.execute())
        } else {
            BeginAcceptedGoal::Cancel(self.begin_cancelling())
        }
    }

    /// Publish feedback for action clients to read.
    pub fn publish_feedback(&self, feedback: &A::Feedback) {
        self.live.publish_feedback(feedback);
    }

    pub(super) fn new(live: Arc<LiveActionServerGoal<A>>) -> Self {
        Self { live }
    }
}

pub enum BeginAcceptedGoal<A: ActionImpl> {
    Execute(ExecutingGoal<A>),
    Cancel(CancellingGoal<A>),
}

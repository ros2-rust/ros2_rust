use super::{CancellingGoal, ExecutingGoal, FeedbackPublisher, LiveActionServerGoal};
use rosidl_runtime_rs::Action;
use std::{future::Future, sync::Arc};

/// This manages a goal which has been accepted but has not begun executing yet.
/// It is allowed to transition into the executing or cancelling state.
pub struct AcceptedGoal<A: Action> {
    live: Arc<LiveActionServerGoal<A>>,
}

impl<A: Action> AcceptedGoal<A> {
    /// Get the goal of this action.
    #[must_use]
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
    /// If the [`Future`] finishes, its output will be provided in [`Ok`].
    ///
    /// If a cancellation request is received before the [`Future`] is finished,
    /// you will receive an [`Err`] with the current state of the [`Future`],
    /// which you can continue processing later if you choose. If you would rather
    /// discard the [`Future`] when a cancellation request is received, you can
    /// call [`Self::unless_cancel_requested`] instead.
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

    /// Process a [`Future`] until it is finished unless a cancellation request
    /// is received.
    ///
    /// If the [`Future`] finishes, its output will be provided in [`Ok`].
    ///
    /// If a cancellation request is received before the [`Future`] is finished,
    /// the [`Future`] will be discarded. This allows non-[`Unpin`] futures to
    /// be passed to this method. If your future implements [`Unpin`] and you want
    /// the option to keep processing it after the cancellation request is received,
    /// then you can call [`Self::until_cancel_requested`] instead.
    ///
    /// After the cancellation request is received, you will still need to trigger
    /// [`Self::begin_cancelling`] or [`Self::reject_cancellation`] to respond to
    /// the request. Otherwise the cancellation request will not receive a response
    /// until the goal reaches a terminal state.
    //
    // TODO(@mxgrey): Add a doctest and example for this.
    pub async fn unless_cancel_requested<F: Future>(&self, f: F) -> Result<F::Output, ()> {
        self.live.cancellation().unless_cancel_requested(f).await
    }

    /// Transition the goal into the cancelling state.
    ///
    /// This does not require an action client to request a cancellation. If no
    /// cancellation was requested, then using this function will have your action
    /// server act as an action client that is commanding that the goal transition
    /// to cancel.
    ///
    /// If there are any open cancellation requests for this goal from any action
    /// clients, they will all be notified that the cancellation is accepted. Any
    /// new cancellation requests that arrive for the goal after this will
    /// automatically be accepted.
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
            BeginAcceptedGoal::Cancel(self.begin_cancelling())
        } else {
            BeginAcceptedGoal::Execute(self.execute())
        }
    }

    /// Publish feedback for action clients to read.
    ///
    /// If you need to publish feedback from a separate thread or async task
    /// which does not have direct access to the goal's state machine, you can
    /// use [`Self::feedback_publisher`] to get a handle that you can pass along.
    pub fn publish_feedback(&self, feedback: A::Feedback) {
        self.live.publish_feedback(feedback);
    }

    /// Get a handle specifically for publishing feedback for this goal. This
    /// publisher can be used separately from the overall state machine of the
    /// goal, but it will stop working once the goal reaches a terminal state.
    ///
    /// If you just need to publish a one-off feedback message, you can use
    /// [`Self::publish_feedback`].
    pub fn feedback_publisher(&self) -> FeedbackPublisher<A> {
        FeedbackPublisher::new(Arc::clone(&self.live))
    }

    pub(super) fn new(live: Arc<LiveActionServerGoal<A>>) -> Self {
        Self { live }
    }
}

/// While a goal was waiting to be executed, it is possible for cancellation
/// requests to arrive for it. An accepted goal may transition directly into
/// either executing or cancelling.
///
/// If you use [`AcceptedGoal::begin`] then the goal will be transitioned into
/// the executing state if no cancellation requests have arrived, or the cancelling
/// state if a cancellation has arrived. This enum will tell you which has occurred.
pub enum BeginAcceptedGoal<A: Action> {
    /// The goal has transitioned into executing.
    Execute(ExecutingGoal<A>),
    /// The goal has transitioned into cancelling.
    Cancel(CancellingGoal<A>),
}

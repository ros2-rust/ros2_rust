use super::{CancellingGoal, FeedbackPublisher, LiveActionServerGoal, TerminatedGoal};
use rosidl_runtime_rs::Action;
use std::{future::Future, sync::Arc};

/// This represents a goal that is in the Executing state. This struct is held
/// by an action server implementation and is used to provide feedback to the
/// client, to listen for cancellation requests, and to transition the goal into
/// its next state.
///
/// If you drop this struct without explicitly transitioning it to its next state,
/// the goal will report itself as aborted.
pub struct ExecutingGoal<A: Action> {
    live: Arc<LiveActionServerGoal<A>>,
}

impl<A: Action> ExecutingGoal<A> {
    /// Get the goal of this action.
    pub fn goal(&self) -> &Arc<A::Goal> {
        self.live.goal()
    }

    /// Transition the goal into the succeeded state.
    ///
    /// "Succeeded" is a terminal state, so the state of the goal can no longer
    /// be changed after this. Publish all relevant feedback before calling this.
    pub fn succeeded_with(self, result: A::Result) -> TerminatedGoal {
        self.live.transition_to_succeed(result);
        TerminatedGoal {
            uuid: *self.live.goal_id(),
        }
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

    /// Transition the goal into the aborted state.
    ///
    /// "Aborted" is a terminal state, so the state of the goal can no longer
    /// be changed after this. Publish all relevant feedback before calling this.
    pub fn aborted_with(self, result: A::Result) -> TerminatedGoal {
        self.live.transition_to_aborted(result);
        TerminatedGoal {
            uuid: *self.live.goal_id(),
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
        live.transition_to_executing();
        Self { live }
    }
}

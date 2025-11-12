use super::{FeedbackPublisher, LiveActionServerGoal, TerminatedGoal};
use rosidl_runtime_rs::Action;
use std::sync::Arc;

/// This represents a goal that is in the Cancelling state. This struct is held
/// by an action server implementation and is used to provide feedback to the
/// client and to transition the goal into its next state. There is no need to
/// listen for cancellation requests since being in this state means all incoming
/// cancellation requests will automatically be accepted.
///
/// If you drop this struct without explicitly transitioning it to its next state,
/// the goal will report itself as aborted.
pub struct CancellingGoal<A: Action> {
    live: Arc<LiveActionServerGoal<A>>,
}

impl<A: Action> CancellingGoal<A> {
    /// Get the goal of this action.
    #[must_use]
    pub fn goal(&self) -> &Arc<A::Goal> {
        self.live.goal()
    }

    /// Terminate the goal with a cancelled state and a specific result value.
    ///
    /// "Cancelled" is a terminal state, so the state of the goal can no longer
    /// be changed after this. Publish all relevant feedback before calling this.
    pub fn cancelled_with(self, result: A::Result) -> TerminatedGoal {
        self.live.transition_to_cancelled(result);
        TerminatedGoal {
            uuid: *self.live.goal_id(),
        }
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
        live.transition_to_cancelling();
        Self { live }
    }
}

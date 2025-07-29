use super::{LiveActionServerGoal, TerminatedGoal};
use std::sync::Arc;
use rosidl_runtime_rs::Action;

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
    pub fn cancelled_with(self, result: &A::Result) -> TerminatedGoal {
        self.live.transition_to_cancelled(result);
        TerminatedGoal { uuid: *self.live.goal_id() }
    }

    /// Transition the goal into the succeeded state.
    ///
    /// "Succeeded" is a terminal state, so the state of the goal can no longer
    /// be changed after this. Publish all relevant feedback before calling this.
    pub fn succeeded_with(self, result: &A::Result) -> TerminatedGoal {
        self.live.transition_to_succeed(result);
        TerminatedGoal { uuid: *self.live.goal_id() }
    }

    /// Transition the goal into the aborted state.
    ///
    /// "Aborted" is a terminal state, so the state of the goal can no longer
    /// be changed after this. Publish all relevant feedback before calling this.
    pub fn aborted_with(self, result: &A::Result) -> TerminatedGoal {
        self.live.transition_to_aborted(result);
        TerminatedGoal { uuid: *self.live.goal_id() }
    }

    /// Publish feedback for action clients to read.
    pub fn publish_feedback(&self, feedback: &A::Feedback) {
        self.live.publish_feedback(feedback);
    }

    pub(super) fn new(live: Arc<LiveActionServerGoal<A>>) -> Self {
        live.transition_to_cancelling();
        Self { live }
    }
}

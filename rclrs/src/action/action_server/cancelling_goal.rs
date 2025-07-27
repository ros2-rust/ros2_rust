use super::LiveActionServerGoal;
use std::sync::Arc;
use rosidl_runtime_rs::ActionImpl;

pub struct CancellingGoal<A: ActionImpl> {
    live: Arc<LiveActionServerGoal<A>>,
}

impl<A: ActionImpl> CancellingGoal<A> {
    /// Terminate the goal with a cancelled state and a specific result value.
    ///
    /// "Cancelled" is a terminal state, so the state of the goal can no longer
    /// be changed after this. Publish all relevant feedback before calling this.
    pub fn cancelled_with(self, result: &A::Result) {
        self.live.transition_to_cancelled(result);
    }

    /// Transition the goal into the succeeded state.
    ///
    /// "Succeeded" is a terminal state, so the state of the goal can no longer
    /// be changed after this. Publish all relevant feedback before calling this.
    pub fn succeeded_with(self, result: &A::Result) {
        self.live.transition_to_succeed(result);
    }

    /// Transition the goal into the aborted state.
    ///
    /// "Aborted" is a terminal state, so the state of the goal can no longer
    /// be changed after this. Publish all relevant feedback before calling this.
    pub fn aborted_with(self, result: &A::Result) {
        self.live.transition_to_aborted(result);
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

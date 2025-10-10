use super::LiveActionServerGoal;
use rosidl_runtime_rs::Action;
use std::sync::Arc;

/// Use this to send feedback from an action server. This struct can be cloned
/// and sent to other threads without the main goal handle. It can continue to
/// send feedback until the goal has reached a terminal state.
#[derive(Clone)]
pub struct FeedbackPublisher<A: Action> {
    live: Arc<LiveActionServerGoal<A>>,
}

impl<A: Action> FeedbackPublisher<A> {
    /// Publish a feedback message, unless the goal has reached a terminal state.
    pub fn publish(&self, feedback: A::Feedback) -> Result<(), A::Feedback> {
        self.live.safe_publish_feedback(feedback)
    }

    pub(super) fn new(live: Arc<LiveActionServerGoal<A>>) -> Self {
        Self { live }
    }
}

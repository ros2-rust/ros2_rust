use crate::log_error;
use super::{CancellingGoal, ExecutingGoal, LiveActionServerGoal};
use std::sync::Arc;
use rosidl_runtime_rs::ActionImpl;

pub struct AcceptedGoal<A: ActionImpl> {
    live: Arc<LiveActionServerGoal<A>>,
}

impl<A: ActionImpl> AcceptedGoal<A> {
    /// Transition the goal into the executing state. This will allow you to
    /// start sending feedback and transition to other states such as cancelling,
    /// succeedded, and aborted.
    ///
    /// You can use this even if an action client has requested a cancellation,
    /// but the cancellation request will be ignored until you handle it with
    /// [`ExecutingGoal`].
    pub fn execute(self) -> ExecutingGoal<A> {
        if let Err(err) = self.live.transition_to_execute() {
            log_error!(
                "AcceptedGoal.execute",
                "Failed to transition to executing status. This error should \
                not happen, please report it to the maintainers of rclrs: {err}",
            );
        }

        ExecutingGoal::new(self.live)
    }

    /// Transition the goal into either the executing mode or the cancelling mode
    /// depending on whether an action client has sent a cancellation request.
    pub fn begin(self) -> BeginAcceptedGoal<A> {

    }

    pub(super) fn new(live: Arc<LiveActionServerGoal<A>>) -> Self {
        Self { live }
    }
}

pub enum BeginAcceptedGoal<A: ActionImpl> {
    Execute(ExecutingGoal<A>),
    Cancel(CancellingGoal<A>),
}

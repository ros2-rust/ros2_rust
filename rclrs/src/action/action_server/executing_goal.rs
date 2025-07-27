use super::LiveActionServerGoal;
use std::sync::Arc;
use rosidl_runtime_rs::ActionImpl;

pub struct ExecutingGoal<A: ActionImpl> {
    live: Arc<LiveActionServerGoal<A>>,
}

impl<A: ActionImpl> ExecutingGoal<A> {
    pub(super) fn new(live: Arc<LiveActionServerGoal<A>>) -> Self {
        Self { live }
    }
}

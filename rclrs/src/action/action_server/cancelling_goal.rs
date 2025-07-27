use super::LiveActionServerGoal;
use std::sync::Arc;
use rosidl_runtime_rs::ActionImpl;

pub struct CancellingGoal<A: ActionImpl> {
    live: Arc<LiveActionServerGoal<A>>,
}


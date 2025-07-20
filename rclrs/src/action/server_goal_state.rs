use std::sync::Arc;
use tokio::sync::watch::{Sender, Receiver};
use rosidl_runtime_rs::ActionImpl;

use crate::{
    ActionServerState, GoalUuid,
};

struct ServerGoalState<A: ActionImpl> {
    cancellation: Arc<CancellationState>,
    server_state: Arc<ActionServerState<A>>,
}

struct CancellationState<A: ActionImpl> {
    receiver: Receiver<CancellationMode<A>>,
    sender: Sender<CancellationMode<A>>,
}

enum CancellationMode<A: ActionImpl> {
    None,
    CancelRequested(CancellationResponder<A>),
    Cancelling,
}

struct CancellationResponder<A: ActionImpl> {
    uuid: GoalUuid,
    handle: Arc<ActionServerState<A>>,
}

impl CancellationResponder {
    fn send_cancel_response(&self) {

    }
}

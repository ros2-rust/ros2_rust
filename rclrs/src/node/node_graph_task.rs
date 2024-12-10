use futures::{
    channel::mpsc::{UnboundedReceiver, UnboundedSender},
    StreamExt,
};

use crate::GuardCondition;

pub(super) enum NodeGraphAction {
    NewGraphListener(UnboundedSender<()>),
    GraphChange,
}

// We take in the GuardCondition to ensure that its Waitable remains in the wait
// set for as long as this task is running. The task will be kept running as long
// as the Node that started it is alive.
pub(super) async fn node_graph_task(
    mut receiver: UnboundedReceiver<NodeGraphAction>,
    #[allow(unused)] guard_condition: GuardCondition,
) {
    let mut listeners = Vec::new();
    while let Some(action) = receiver.next().await {
        match action {
            NodeGraphAction::NewGraphListener(listener) => {
                if listener.unbounded_send(()).is_ok() {
                    // The listener might or might not still be relevant, so
                    // keep it until we see that the receiver is dropped.
                    listeners.push(listener);
                }
            }
            NodeGraphAction::GraphChange => {
                // We should let all listeners know that a graph event happened.
                // If we see that the listener's receiver has dropped (i.e.
                // unbounded_send returns an Err) then we remove it from the
                // container.
                listeners.retain(|listener| listener.unbounded_send(()).is_ok());
            }
        }
    }
}

use rosidl_runtime_rs::Message;

use crate::{
    AnySubscriptionCallback, ExecutorCommands, SubscriptionHandle,
};

use futures::{
    Stream, StreamExt,
    channel::mpsc::UnboundedReceiver
};

use std::sync::Arc;

pub(super) enum SubscriptionAction<T: Message> {
    Execute,
    SetCallback(AnySubscriptionCallback<T>),
}

pub(super) async fn subscription_task<T: Message>(
    mut callback: AnySubscriptionCallback<T>,
    mut receiver: UnboundedReceiver<SubscriptionAction<T>>,
    handle: Arc<SubscriptionHandle>,
    commands: Arc<ExecutorCommands>,
) {
    while let Some(action) = receiver.next().await {
        match action {
            SubscriptionAction::SetCallback(new_callback) => {
                callback = new_callback;
            }
            SubscriptionAction::Execute => {
                if let Err(_) = callback.execute(&handle, &commands) {
                    // TODO(@mxgrey): Log the error here once logging is implemented
                }
            }
        }
    }
}

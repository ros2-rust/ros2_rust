use rosidl_runtime_rs::Service;

use crate::{
    service::ServiceHandle,
    AnyServiceCallback, ExecutorCommands,
};

use futures::{
    channel::mpsc::UnboundedReceiver,
    StreamExt,
};

use std::sync::Arc;

pub(super) enum ServiceAction<T: Service> {
    Execute,
    SetCallback(AnyServiceCallback<T>),
}

pub(super) async fn service_task<T: Service>(
    mut callback: AnyServiceCallback<T>,
    mut receiver: UnboundedReceiver<ServiceAction<T>>,
    handle: Arc<ServiceHandle>,
    commands: Arc<ExecutorCommands>,
) {
    while let Some(action) = receiver.next().await {
        match action {
            ServiceAction::SetCallback(new_callback) => {
                callback = new_callback;
            }
            ServiceAction::Execute => {
                if let Err(_) = callback.execute(&handle, &commands) {
                    // TODO(@mxgrey): Log the error here once logging is implemented
                }
            }
        }
    }
}

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
    dbg!();
    while let Some(action) = receiver.next().await {
        match action {
            ServiceAction::SetCallback(new_callback) => {
                dbg!();
                callback = new_callback;
            }
            ServiceAction::Execute => {
                dbg!();
                if let Err(err) = callback.execute(&handle, &commands) {
                    // TODO(@mxgrey): Log the error here once logging is implemented
                    eprintln!("Error while executing a service callback: {err}");
                }
            }
        }
    }
}

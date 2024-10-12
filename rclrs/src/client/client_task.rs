use rosidl_runtime_rs::{Message, Service};

use futures::{
    channel::mpsc::UnboundedReceiver,
    StreamExt,
};

use std::{
    sync::Arc,
    collections::HashMap,
};

use crate::{
    error::ToResult,
    rcl_bindings::*,
    client::ClientHandle,
    AnyClientOutputSender, ServiceInfo,
    RclrsError,
};

pub enum ClientAction<T: Service> {
    TakeResponse,
    NewRequest{
        sequence_number: SequenceNumber,
        sender: AnyClientOutputSender<T::Response>,
    },
}

pub(super) type SequenceNumber = i64;

pub(super) async fn client_task<T: Service>(
    mut receiver: UnboundedReceiver<ClientAction<T>>,
    handle: Arc<ClientHandle>,
) {
    // This stores all active requests that have not received a response yet
    let mut active_requests: HashMap<SequenceNumber, AnyClientOutputSender<T::Response>> = HashMap::new();

    // This holds responses that came in when no active request matched the
    // sequence number. This could happen if the TakeResponse arrives before the
    // NewRequest for the same sequence number. That is extremely unlikely to
    // ever happen but is theoretically possible, so we should account for it.
    let mut loose_responses: HashMap<SequenceNumber, (T::Response, rmw_service_info_t)> = HashMap::new();

    while let Some(action) = receiver.next().await {
        match action {
            ClientAction::TakeResponse => {
                match take_response::<T::Response>(&handle) {
                    Ok((response, info)) => {
                        let seq = info.request_id.sequence_number;
                        if let Some(sender) = active_requests.remove(&seq) {
                            // The active request is available, so send this response off
                            sender.send_response(response, info);
                        } else {
                            // Weirdly there isn't an active request for this, so save
                            // it in the loose responses map.
                            loose_responses.insert(seq, (response, info));
                        }
                    }
                    Err(_err) => {
                        // TODO(@mxgrey): Log the error here once logging is available
                    }
                }
            }
            ClientAction::NewRequest { sequence_number, sender } => {
                if let Some((response, info)) = loose_responses.remove(&sequence_number) {
                    // The response for this request already arrive, so we'll
                    // send it off immediately.
                    sender.send_response(response, info);
                } else {
                    active_requests.insert(sequence_number, sender);
                }
            }
        }
    }
}

fn take_response<Response: Message>(
    handle: &Arc<ClientHandle>,
) -> Result<(Response, rmw_service_info_t), RclrsError> {
    let mut service_info_out = ServiceInfo::zero_initialized_rmw();
    let mut response_out = Response::RmwMsg::default();
    let handle = &*handle.lock();
    unsafe {
        // SAFETY: The three pointers are all kept valid by the handle
        rcl_take_response_with_info(
            handle,
            &mut service_info_out,
            &mut response_out as *mut Response::RmwMsg as *mut _,
        )
    }
    .ok()
    .map(|_| (
        Response::from_rmw_message(response_out),
        service_info_out,
    ))
}

use rosidl_runtime_rs::Message;

use futures::channel::oneshot::{Sender, channel};

use crate::{
    rcl_bindings::rmw_service_info_t,
    RequestId, ServiceInfo, Promise,
};

/// This trait allows us to deduce how much information a user wants to receive
/// from a client call. A user can choose to receive only the response from the
/// service or may include the [`RequestId`] or [`ServiceInfo`] metadata.
///
/// Users never need to use this trait directly.
pub trait ClientOutput<Response>: Sized {
    /// Create the appropriate type of channel to send the information that the
    /// user asked for.
    fn create_channel() -> (AnyClientOutputSender<Response>, Promise<Self>);
}

impl<Response: Message> ClientOutput<Response> for Response {
    fn create_channel() -> (AnyClientOutputSender<Response>, Promise<Self>) {
        let (sender, receiver) = channel();
        (AnyClientOutputSender::ResponseOnly(sender), receiver)
    }
}

impl<Response: Message> ClientOutput<Response> for (Response, RequestId) {
    fn create_channel() -> (AnyClientOutputSender<Response>, Promise<Self>) {
        let (sender, receiver) = channel();
        (AnyClientOutputSender::WithId(sender), receiver)
    }
}

impl<Response: Message> ClientOutput<Response> for (Response, ServiceInfo) {
    fn create_channel() -> (AnyClientOutputSender<Response>, Promise<Self>) {
        let (sender, receiver) = channel();
        (AnyClientOutputSender::WithServiceInfo(sender), receiver)
    }
}

/// Can send any kind of response for a client call.
pub enum AnyClientOutputSender<Response> {
    /// The user only asked for the response.
    ResponseOnly(Sender<Response>),
    /// The user also asked for the RequestId
    WithId(Sender<(Response, RequestId)>),
    /// The user also asked for the ServiceInfo
    WithServiceInfo(Sender<(Response, ServiceInfo)>),
}

impl<Response: Message> AnyClientOutputSender<Response> {
    pub(super) fn send_response(
        self,
        response: Response,
        service_info: rmw_service_info_t,
    ) {
        match self {
            Self::ResponseOnly(sender) => {
                let _ = sender.send(response);
            }
            Self::WithId(sender) => {
                let _ = sender.send((
                    response,
                    RequestId::from_rmw_request_id(&service_info.request_id),
                ));
            }
            Self::WithServiceInfo(sender) => {
                let _ = sender.send((
                    response,
                    ServiceInfo::from_rmw_service_info(&service_info),
                ));
            }
        }
    }
}

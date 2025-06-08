use rosidl_runtime_rs::Service;

use super::{AnyServiceCallback, NodeServiceCallback, RequestId, ServiceInfo};

use std::future::Future;

/// A trait for async callbacks of services.
///
/// Three callback signatures are supported:
/// - [`FnMut`] ( `Request` ) -> impl [`Future`]<Output=`Response`>
/// - [`FnMut`] ( `Request`, [`RequestId`] ) -> impl [`Future`]<Output=`Response`>
/// - [`FnMut`] ( `Request`, [`ServiceInfo`] ) -> impl [`Future`]<Output=`Response`>
pub trait IntoAsyncServiceCallback<T, Args>: Send + 'static
where
    T: Service,
{
    /// Converts the callback into an enum.
    ///
    /// User code never needs to call this function.
    fn into_async_service_callback(self) -> AnyServiceCallback<T, ()>;
}

impl<T, F, Func> IntoAsyncServiceCallback<T, ()> for Func
where
    T: Service,
    Func: FnMut(T::Request) -> F + Send + 'static,
    F: Future<Output = T::Response> + Send + 'static,
{
    fn into_async_service_callback(mut self) -> AnyServiceCallback<T, ()> {
        NodeServiceCallback::OnlyRequest(Box::new(move |request| Box::pin(self(request)))).into()
    }
}

impl<T, F, Func> IntoAsyncServiceCallback<T, RequestId> for Func
where
    T: Service,
    Func: FnMut(T::Request, RequestId) -> F + Send + 'static,
    F: Future<Output = T::Response> + Send + 'static,
{
    fn into_async_service_callback(mut self) -> AnyServiceCallback<T, ()> {
        NodeServiceCallback::WithId(Box::new(move |request, request_id| {
            Box::pin(self(request, request_id))
        }))
        .into()
    }
}

impl<T, F, Func> IntoAsyncServiceCallback<T, ServiceInfo> for Func
where
    T: Service,
    Func: FnMut(T::Request, ServiceInfo) -> F + Send + 'static,
    F: Future<Output = T::Response> + Send + 'static,
{
    fn into_async_service_callback(mut self) -> AnyServiceCallback<T, ()> {
        NodeServiceCallback::WithInfo(Box::new(move |request, service_info| {
            Box::pin(self(request, service_info))
        }))
        .into()
    }
}

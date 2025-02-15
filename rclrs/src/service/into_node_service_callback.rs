use rosidl_runtime_rs::Service;

use crate::{AnyServiceCallback, NodeServiceCallback, RequestId, ServiceInfo};

use std::sync::Arc;

/// A trait to deduce regular callbacks of services.
///
/// Users of rclrs never need to use this trait directly.
//
// TODO(@mxgrey): Add a description of what callbacks signatures are supported
pub trait IntoNodeServiceCallback<T, Args>: Send + 'static
where
    T: Service,
{
    /// Converts the callback into an enum.
    ///
    /// User code never needs to call this function.
    fn into_node_service_callback(self) -> AnyServiceCallback<T, ()>;
}

impl<T, Func> IntoNodeServiceCallback<T, ()> for Func
where
    T: Service,
    Func: Fn(T::Request) -> T::Response + Send + Sync + 'static,
{
    fn into_node_service_callback(self) -> AnyServiceCallback<T, ()> {
        let func = Arc::new(self);
        NodeServiceCallback::OnlyRequest(Box::new(move |request| {
            let f = Arc::clone(&func);
            Box::pin(async move { f(request) })
        }))
        .into()
    }
}

impl<T, Func> IntoNodeServiceCallback<T, RequestId> for Func
where
    T: Service,
    Func: Fn(T::Request, RequestId) -> T::Response + Send + Sync + 'static,
{
    fn into_node_service_callback(self) -> AnyServiceCallback<T, ()> {
        let func = Arc::new(self);
        NodeServiceCallback::WithId(Box::new(move |request, request_id| {
            let f = Arc::clone(&func);
            Box::pin(async move { f(request, request_id) })
        }))
        .into()
    }
}

impl<T, Func> IntoNodeServiceCallback<T, ServiceInfo> for Func
where
    T: Service,
    Func: Fn(T::Request, ServiceInfo) -> T::Response + Send + Sync + 'static,
{
    fn into_node_service_callback(self) -> AnyServiceCallback<T, ()> {
        let func = Arc::new(self);
        NodeServiceCallback::WithInfo(Box::new(move |request, service_info| {
            let f = Arc::clone(&func);
            Box::pin(async move { f(request, service_info) })
        }))
        .into()
    }
}

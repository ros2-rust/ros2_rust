use rosidl_runtime_rs::Service;

use crate::{RequestId, ServiceInfo};

/// A trait to deduce regular callbacks of service clients.
///
/// Users of rclrs never need to use this trait directly.
//
// TODO(@mxgrey): Add a description of what callback signatures are supported
pub trait ClientCallback<T, Args>: Send + 'static
where
    T: Service,
{
    /// Trigger the callback to run
    fn run_client_callback(self, response: T::Response, info: ServiceInfo);
}

impl<T, Func> ClientCallback<T, ()> for Func
where
    T: Service,
    Func: FnOnce(T::Response) + Send + 'static,
{
    fn run_client_callback(self, response: T::Response, _info: ServiceInfo) {
        self(response)
    }
}

impl<T, Func> ClientCallback<T, RequestId> for Func
where
    T: Service,
    Func: FnOnce(T::Response, RequestId) + Send + 'static,
{
    fn run_client_callback(self, response: T::Response, info: ServiceInfo) {
        self(response, info.request_id)
    }
}

impl<T, Func> ClientCallback<T, ServiceInfo> for Func
where
    T: Service,
    Func: FnOnce(T::Response, ServiceInfo) + Send + 'static,
{
    fn run_client_callback(self, response: T::Response, info: ServiceInfo) {
        self(response, info)
    }
}

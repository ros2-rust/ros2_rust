use rosidl_runtime_rs::Service;

use std::future::Future;

use crate::{ServiceInfo, RequestId};

/// A trait to deduce async callbacks of service clients.
///
/// Users of rclrs never need to use this trait directly.
//
// TODO(@mxgrey): Add a description of what callback signatures are supported
pub trait ClientAsyncCallback<T, Args>: Send + 'static
where
    T: Service,
{
    /// This represents the type of task (Future) that will be produced by the callback
    type Task: Future<Output = ()> + Send;

    /// Trigger the callback to run
    fn run_client_async_callback(self, response: T::Response, info: ServiceInfo) -> Self::Task;
}

impl<T, Func, Fut> ClientAsyncCallback<T, ()> for Func
where
    T: Service,
    Func: FnOnce(T::Response) -> Fut + Send + 'static,
    Fut: Future<Output = ()> + Send,
{
    type Task = Fut;
    fn run_client_async_callback(self, response: T::Response, _info: ServiceInfo) -> Fut {
        self(response)
    }
}

impl<T, Func, Fut> ClientAsyncCallback<T, RequestId> for Func
where
    T: Service,
    Func: FnOnce(T::Response, RequestId) -> Fut + Send + 'static,
    Fut: Future<Output = ()> + Send,
{
    type Task = Fut;
    fn run_client_async_callback(self, response: T::Response, info: ServiceInfo) -> Fut {
        self(response, info.request_id)
    }
}

impl<T, Func, Fut> ClientAsyncCallback<T, ServiceInfo> for Func
where
    T: Service,
    Func: FnOnce(T::Response, ServiceInfo) -> Fut + Send + 'static,
    Fut: Future<Output = ()> + Send,
{
    type Task = Fut;
    fn run_client_async_callback(self, response: T::Response, info: ServiceInfo) -> Fut {
        self(response, info)
    }
}

use std::{
    boxed::Box,
    ffi::CString,
    sync::{atomic::AtomicBool, Arc, Mutex, MutexGuard},
};

use rosidl_runtime_rs::Message;

use crate::{
    error::{RclReturnCode, ToResult},
    rcl_bindings::*,
    MessageCow, NodeHandle, RclrsError, ENTITY_LIFECYCLE_MUTEX,
};

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_service_t {}

/// Manage the lifecycle of an `rcl_service_t`, including managing its dependencies
/// on `rcl_node_t` and `rcl_context_t` by ensuring that these dependencies are
/// [dropped after][1] the `rcl_service_t`.
///
/// [1]: <https://doc.rust-lang.org/reference/destructors.html>
pub struct ServiceHandle {
    rcl_service: Mutex<rcl_service_t>,
    node_handle: Arc<NodeHandle>,
    pub(crate) in_use_by_wait_set: Arc<AtomicBool>,
}

impl ServiceHandle {
    pub(crate) fn lock(&self) -> MutexGuard<rcl_service_t> {
        self.rcl_service.lock().unwrap()
    }
}

impl Drop for ServiceHandle {
    fn drop(&mut self) {
        let rcl_service = self.rcl_service.get_mut().unwrap();
        let mut rcl_node = self.node_handle.rcl_node.lock().unwrap();
        let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
        // SAFETY: The entity lifecycle mutex is locked to protect against the risk of
        // global variables in the rmw implementation being unsafely modified during cleanup.
        unsafe {
            rcl_service_fini(rcl_service, &mut **rcl_node);
        }
    }
}

/// Trait to be implemented by concrete Service structs.
///
/// See [`Service<T>`] for an example
pub trait ServiceBase: Send + Sync {
    /// Internal function to get a reference to the `rcl` handle.
    fn handle(&self) -> &ServiceHandle;
    /// Tries to take a new request and run the callback with it.
    fn execute(&self) -> Result<(), RclrsError>;
}

type ServiceCallback<Request, Response> =
    Box<dyn Fn(&rmw_request_id_t, Request) -> Response + 'static + Send>;

/// Main class responsible for responding to requests sent by ROS clients.
///
/// The only available way to instantiate services is via [`Node::create_service()`][1], this is to
/// ensure that [`Node`][2]s can track all the services that have been created.
///
/// [1]: crate::Node::create_service
/// [2]: crate::Node
pub struct Service<T>
where
    T: rosidl_runtime_rs::Service,
{
    pub(crate) handle: Arc<ServiceHandle>,
    /// The callback function that runs when a request was received.
    pub callback: Mutex<ServiceCallback<T::Request, T::Response>>,
}

impl<T> Service<T>
where
    T: rosidl_runtime_rs::Service,
{
    /// Creates a new service.
    pub(crate) fn new<F>(
        node_handle: Arc<NodeHandle>,
        topic: &str,
        callback: F,
    ) -> Result<Self, RclrsError>
    // This uses pub(crate) visibility to avoid instantiating this struct outside
    // [`Node::create_service`], see the struct's documentation for the rationale
    where
        T: rosidl_runtime_rs::Service,
        F: Fn(&rmw_request_id_t, T::Request) -> T::Response + 'static + Send,
    {
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_service = unsafe { rcl_get_zero_initialized_service() };
        let type_support = <T as rosidl_runtime_rs::Service>::get_type_support()
            as *const rosidl_service_type_support_t;
        let topic_c_string = CString::new(topic).map_err(|err| RclrsError::StringContainsNul {
            err,
            s: topic.into(),
        })?;

        // SAFETY: No preconditions for this function.
        let service_options = unsafe { rcl_service_get_default_options() };

        {
            let rcl_node = node_handle.rcl_node.lock().unwrap();
            let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
            unsafe {
                // SAFETY:
                // * The rcl_service is zero-initialized as mandated by this function.
                // * The rcl_node is kept alive by the NodeHandle it is a dependency of the service.
                // * The topic name and the options are copied by this function, so they can be dropped
                //   afterwards.
                // * The entity lifecycle mutex is locked to protect against the risk of global
                //   variables in the rmw implementation being unsafely modified during initialization.
                rcl_service_init(
                    &mut rcl_service,
                    &**rcl_node,
                    type_support,
                    topic_c_string.as_ptr(),
                    &service_options as *const _,
                )
                .ok()?;
            }
        }

        let handle = Arc::new(ServiceHandle {
            rcl_service: Mutex::new(rcl_service),
            node_handle,
            in_use_by_wait_set: Arc::new(AtomicBool::new(false)),
        });

        Ok(Self {
            handle,
            callback: Mutex::new(Box::new(callback)),
        })
    }

    /// Fetches a new request.
    ///
    /// When there is no new message, this will return a
    /// [`ServiceTakeFailed`][1].
    ///
    /// [1]: crate::RclrsError
    //
    // ```text
    // +---------------------+
    // | rclrs::take_request |
    // +----------+----------+
    //            |
    //            |
    // +----------v----------+
    // |  rcl_take_request   |
    // +----------+----------+
    //            |
    //            |
    // +----------v----------+
    // |      rmw_take       |
    // +---------------------+
    // ```
    pub fn take_request(&self) -> Result<(T::Request, rmw_request_id_t), RclrsError> {
        let mut request_id_out = rmw_request_id_t {
            writer_guid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            sequence_number: 0,
        };
        type RmwMsg<T> =
            <<T as rosidl_runtime_rs::Service>::Request as rosidl_runtime_rs::Message>::RmwMsg;
        let mut request_out = RmwMsg::<T>::default();
        let handle = &*self.handle.lock();
        unsafe {
            // SAFETY: The three pointers are valid/initialized
            rcl_take_request(
                handle,
                &mut request_id_out,
                &mut request_out as *mut RmwMsg<T> as *mut _,
            )
        }
        .ok()?;
        Ok((T::Request::from_rmw_message(request_out), request_id_out))
    }
}

impl<T> ServiceBase for Service<T>
where
    T: rosidl_runtime_rs::Service,
{
    fn handle(&self) -> &ServiceHandle {
        &self.handle
    }

    fn execute(&self) -> Result<(), RclrsError> {
        let (req, mut req_id) = match self.take_request() {
            Ok((req, req_id)) => (req, req_id),
            Err(RclrsError::RclError {
                code: RclReturnCode::ServiceTakeFailed,
                ..
            }) => {
                // Spurious wakeup – this may happen even when a waitset indicated that this
                // service was ready, so it shouldn't be an error.
                return Ok(());
            }
            Err(e) => return Err(e),
        };
        let res = (*self.callback.lock().unwrap())(&req_id, req);
        let rmw_message = <T::Response as Message>::into_rmw_message(res.into_cow());
        let handle = &*self.handle.lock();
        unsafe {
            // SAFETY: The response type is guaranteed to match the service type by the type system.
            rcl_send_response(
                handle,
                &mut req_id,
                rmw_message.as_ref() as *const <T::Response as Message>::RmwMsg as *mut _,
            )
        }
        .ok()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_helpers::*;

    #[test]
    fn traits() {
        assert_send::<Service<test_msgs::srv::Arrays>>();
        assert_sync::<Service<test_msgs::srv::Arrays>>();
    }

    #[test]
    fn test_services() -> Result<(), RclrsError> {
        use crate::TopicNamesAndTypes;
        use test_msgs::srv;

        let namespace = "/test_services_graph";
        let graph = construct_test_graph(namespace)?;
        let check_names_and_types = |names_and_types: TopicNamesAndTypes| {
            let types = names_and_types
                .get("/test_services_graph/graph_test_topic_4")
                .unwrap();
            assert!(types.contains(&"test_msgs/srv/Empty".to_string()));
        };

        let _node_1_empty_service =
            graph
                .node1
                .create_service::<srv::Empty, _>("graph_test_topic_4", |_, _| {
                    srv::Empty_Response {
                        structure_needs_at_least_one_member: 0,
                    }
                })?;
        let _node_2_empty_client = graph
            .node2
            .create_client::<srv::Empty>("graph_test_topic_4")?;

        std::thread::sleep(std::time::Duration::from_millis(100));

        let service_names_and_types = graph.node1.get_service_names_and_types()?;
        check_names_and_types(service_names_and_types);

        let service_names_and_types = graph
            .node1
            .get_service_names_and_types_by_node(&graph.node1.name(), namespace)?;
        check_names_and_types(service_names_and_types);

        Ok(())
    }
}

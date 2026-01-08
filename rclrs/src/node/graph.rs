use std::{
    collections::HashMap,
    ffi::{CStr, CString},
};

use crate::{rcl_bindings::*, NodeState, RclrsError, ToResult};

impl Drop for rmw_names_and_types_t {
    fn drop(&mut self) {
        // SAFETY: No preconditions for this function.
        unsafe {
            rcl_names_and_types_fini(self).ok().unwrap();
        }
    }
}

impl Drop for rmw_topic_endpoint_info_array_t {
    fn drop(&mut self) {
        // SAFETY: No preconditions for this function.
        unsafe {
            rmw_topic_endpoint_info_array_fini(self, &mut rcutils_get_default_allocator())
                .ok()
                .unwrap();
        }
    }
}

impl Drop for rcutils_string_array_t {
    fn drop(&mut self) {
        // SAFETY: No preconditions for this function.
        unsafe {
            rcutils_string_array_fini(self);
        }
    }
}

/// Stores a list of types associated with each topic.
pub type TopicNamesAndTypes = HashMap<String, Vec<String>>;

/// Stores a node's name and namespace
#[derive(Debug, PartialEq, Eq)]
pub struct NodeNameInfo {
    /// The name of the node
    pub name: String,
    /// The namespace of the node
    pub namespace: String,
}

/// Contains topic endpoint information
#[derive(Debug, PartialEq, Eq)]
pub struct TopicEndpointInfo {
    /// The name of the endpoint node
    pub node_name: String,
    /// The namespace of the endpoint node
    pub node_namespace: String,
    /// The type of the topic
    pub topic_type: String,
}

impl NodeState {
    /// Returns a list of topic names and types for publishers associated with a node.
    pub fn get_publisher_names_and_types_by_node(
        &self,
        node: &str,
        namespace: &str,
    ) -> Result<TopicNamesAndTypes, RclrsError> {
        // SAFETY: Forwarding arguments to the inner C function is safe
        unsafe extern "C" fn wrapper(
            node: *const rcl_node_t,
            allocator: *mut rcl_allocator_t,
            node_name: *const ::std::os::raw::c_char,
            node_namespace: *const ::std::os::raw::c_char,
            topic_names_and_types: *mut rcl_names_and_types_t,
        ) -> rcl_ret_t {
            rcl_get_publisher_names_and_types_by_node(
                node,
                allocator,
                false,
                node_name,
                node_namespace,
                topic_names_and_types,
            )
        }

        self.get_names_and_types_by_node(node, namespace, wrapper)
    }

    /// Returns a list of topic names and types for subscriptions associated with a node.
    pub fn get_subscription_names_and_types_by_node(
        &self,
        node: &str,
        namespace: &str,
    ) -> Result<TopicNamesAndTypes, RclrsError> {
        // SAFETY: Forwarding arguments to the inner C function is safe
        unsafe extern "C" fn wrapper(
            node: *const rcl_node_t,
            allocator: *mut rcl_allocator_t,
            node_name: *const ::std::os::raw::c_char,
            node_namespace: *const ::std::os::raw::c_char,
            topic_names_and_types: *mut rcl_names_and_types_t,
        ) -> rcl_ret_t {
            rcl_get_subscriber_names_and_types_by_node(
                node,
                allocator,
                false,
                node_name,
                node_namespace,
                topic_names_and_types,
            )
        }

        self.get_names_and_types_by_node(node, namespace, wrapper)
    }

    /// Returns a list of topic names and types for services associated with a node.
    pub fn get_service_names_and_types_by_node(
        &self,
        node: &str,
        namespace: &str,
    ) -> Result<TopicNamesAndTypes, RclrsError> {
        self.get_names_and_types_by_node(node, namespace, rcl_get_service_names_and_types_by_node)
    }

    /// Returns a list of topic names and types for clients associated with a node.
    pub fn get_client_names_and_types_by_node(
        &self,
        node: &str,
        namespace: &str,
    ) -> Result<TopicNamesAndTypes, RclrsError> {
        self.get_names_and_types_by_node(node, namespace, rcl_get_client_names_and_types_by_node)
    }

    /// Returns a list of all topic names and their types.
    pub fn get_topic_names_and_types(&self) -> Result<TopicNamesAndTypes, RclrsError> {
        // SAFETY: Getting a zero-initialized value is always safe
        let mut rcl_names_and_types = unsafe { rmw_get_zero_initialized_names_and_types() };

        // SAFETY: rcl_names_and_types is zero-initialized as expected by this call
        unsafe {
            let rcl_node = self.handle.rcl_node.lock().unwrap();
            rcl_get_topic_names_and_types(
                &*rcl_node,
                &mut rcutils_get_default_allocator(),
                false,
                &mut rcl_names_and_types,
            )
            .ok()?
        };

        Ok(convert_names_and_types(rcl_names_and_types))
    }

    /// Returns a list of service names and types for this node.
    pub fn get_service_names_and_types(&self) -> Result<TopicNamesAndTypes, RclrsError> {
        self.get_service_names_and_types_by_node(&self.name(), &self.namespace())
    }

    /// Returns a list of all node names.
    pub fn get_node_names(&self) -> Result<Vec<NodeNameInfo>, RclrsError> {
        // SAFETY: Getting a zero-initialized value is always safe
        let (mut rcl_names, mut rcl_namespaces) = unsafe {
            (
                rcutils_get_zero_initialized_string_array(),
                rcutils_get_zero_initialized_string_array(),
            )
        };

        // SAFETY: node_names and node_namespaces are zero-initialized as expected by this call.
        unsafe {
            let rcl_node = self.handle.rcl_node.lock().unwrap();
            rcl_get_node_names(
                &*rcl_node,
                rcutils_get_default_allocator(),
                &mut rcl_names,
                &mut rcl_namespaces,
            )
            .ok()?;
        };

        // SAFETY: Because the previous function call successfully returned, the names and
        // namespaces are populated with valid data
        let (names_slice, namespaces_slice) = unsafe {
            (
                rcl_from_raw_parts(rcl_names.data, rcl_names.size),
                rcl_from_raw_parts(rcl_namespaces.data, rcl_namespaces.size),
            )
        };

        // SAFETY: Because rcl_get_node_names successfully returned, the name and namespace pointers
        // point to valid C strings
        let zipped_names = names_slice
            .iter()
            .zip(namespaces_slice.iter())
            .map(|(name, namespace)| unsafe {
                NodeNameInfo {
                    name: CStr::from_ptr(*name).to_string_lossy().into_owned(),
                    namespace: CStr::from_ptr(*namespace).to_string_lossy().into_owned(),
                }
            })
            .collect();

        Ok(zipped_names)
    }

    /// Returns a list of all node names with enclaves.
    pub fn get_node_names_with_enclaves(&self) -> Result<Vec<(NodeNameInfo, String)>, RclrsError> {
        // SAFETY: Getting a zero-initialized value is always safe
        let (mut rcl_names, mut rcl_namespaces, mut rcl_enclaves) = unsafe {
            (
                rcutils_get_zero_initialized_string_array(),
                rcutils_get_zero_initialized_string_array(),
                rcutils_get_zero_initialized_string_array(),
            )
        };

        // SAFETY: The node_names, namespaces, and enclaves are zero-initialized as expected by this call.
        unsafe {
            let rcl_node = self.handle.rcl_node.lock().unwrap();
            rcl_get_node_names_with_enclaves(
                &*rcl_node,
                rcutils_get_default_allocator(),
                &mut rcl_names,
                &mut rcl_namespaces,
                &mut rcl_enclaves,
            )
            .ok()?;
        };

        // SAFETY: The previous function successfully returned, so the arrays are valid
        let (names_slice, namespaces_slice, enclaves_slice) = unsafe {
            (
                rcl_from_raw_parts(rcl_names.data, rcl_names.size),
                rcl_from_raw_parts(rcl_namespaces.data, rcl_namespaces.size),
                rcl_from_raw_parts(rcl_enclaves.data, rcl_enclaves.size),
            )
        };

        // SAFETY: The rcl function call successfully returned, so each element of the arrays points to
        // a valid C string
        let zipped_names = names_slice
            .iter()
            .zip(namespaces_slice.iter())
            .zip(enclaves_slice.iter())
            .map(|((name, namespace), enclave)| unsafe {
                (
                    NodeNameInfo {
                        name: CStr::from_ptr(*name).to_string_lossy().into_owned(),
                        namespace: CStr::from_ptr(*namespace).to_string_lossy().into_owned(),
                    },
                    CStr::from_ptr(*enclave).to_string_lossy().into_owned(),
                )
            })
            .collect();

        Ok(zipped_names)
    }

    /// Counts the number of publishers for a given topic.
    pub fn count_publishers(&self, topic: &str) -> Result<usize, RclrsError> {
        let topic_name = CString::new(topic).map_err(|err| RclrsError::StringContainsNul {
            s: topic.to_string(),
            err,
        })?;
        let mut count: usize = 0;

        // SAFETY: The topic_name string was correctly allocated previously
        unsafe {
            let rcl_node = self.handle.rcl_node.lock().unwrap();
            rcl_count_publishers(&*rcl_node, topic_name.as_ptr(), &mut count).ok()?
        };
        Ok(count)
    }

    /// Counts the number of subscriptions for a given topic.
    pub fn count_subscriptions(&self, topic: &str) -> Result<usize, RclrsError> {
        let topic_name = CString::new(topic).map_err(|err| RclrsError::StringContainsNul {
            s: topic.to_string(),
            err,
        })?;
        let mut count: usize = 0;

        // SAFETY: The topic_name string was correctly allocated previously
        unsafe {
            let rcl_node = self.handle.rcl_node.lock().unwrap();
            rcl_count_subscribers(&*rcl_node, topic_name.as_ptr(), &mut count).ok()?
        };
        Ok(count)
    }

    /// Returns topic publisher info.
    pub fn get_publishers_info_by_topic(
        &self,
        topic: &str,
    ) -> Result<Vec<TopicEndpointInfo>, RclrsError> {
        self.get_publisher_subscriber_info_by_topic(topic, rcl_get_publishers_info_by_topic)
    }

    /// Returns topic subscriptions info.
    pub fn get_subscriptions_info_by_topic(
        &self,
        topic: &str,
    ) -> Result<Vec<TopicEndpointInfo>, RclrsError> {
        self.get_publisher_subscriber_info_by_topic(topic, rcl_get_subscriptions_info_by_topic)
    }

    /// Returns an rcl names_and_types function, without a "no_demangle" argument.
    fn get_names_and_types_by_node(
        &self,
        node: &str,
        namespace: &str,
        getter: unsafe extern "C" fn(
            *const rcl_node_t,
            *mut rcl_allocator_t,
            *const ::std::os::raw::c_char,
            *const ::std::os::raw::c_char,
            *mut rcl_names_and_types_t,
        ) -> rcl_ret_t,
    ) -> Result<TopicNamesAndTypes, RclrsError> {
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_names_and_types = unsafe { rmw_get_zero_initialized_names_and_types() };

        let node_name = CString::new(node).map_err(|err| RclrsError::StringContainsNul {
            s: node.to_string(),
            err,
        })?;
        let node_namespace =
            CString::new(namespace).map_err(|err| RclrsError::StringContainsNul {
                s: namespace.to_string(),
                err,
            })?;

        // SAFETY: node_name and node_namespace have been zero-initialized.
        unsafe {
            let rcl_node = self.handle.rcl_node.lock().unwrap();
            getter(
                &*rcl_node,
                &mut rcutils_get_default_allocator(),
                node_name.as_ptr(),
                node_namespace.as_ptr(),
                &mut rcl_names_and_types,
            )
        };

        Ok(convert_names_and_types(rcl_names_and_types))
    }

    /// Returns publisher or subscriber info by topic.
    fn get_publisher_subscriber_info_by_topic(
        &self,
        topic: &str,
        getter: unsafe extern "C" fn(
            *const rcl_node_t,
            *mut rcl_allocator_t,
            *const ::std::os::raw::c_char,
            bool,
            *mut rcl_topic_endpoint_info_array_t,
        ) -> rcl_ret_t,
    ) -> Result<Vec<TopicEndpointInfo>, RclrsError> {
        let topic = CString::new(topic).map_err(|err| RclrsError::StringContainsNul {
            s: topic.to_string(),
            err,
        })?;

        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_publishers_info =
            unsafe { rmw_get_zero_initialized_topic_endpoint_info_array() };

        // SAFETY: topic has been zero-initialized
        unsafe {
            let rcl_node = self.handle.rcl_node.lock().unwrap();
            getter(
                &*rcl_node,
                &mut rcutils_get_default_allocator(),
                topic.as_ptr(),
                false,
                &mut rcl_publishers_info,
            )
            .ok()?;
        }

        // SAFETY: The previous call returned successfully, so the data is valid
        let topic_endpoint_infos_slice =
            unsafe { rcl_from_raw_parts(rcl_publishers_info.info_array, rcl_publishers_info.size) };

        // SAFETY: Because the rcl call returned successfully, each element of the slice points
        // to a valid topic_endpoint_info object, which contains valid C strings
        let topic_endpoint_infos_vec = topic_endpoint_infos_slice
            .iter()
            .map(|info| {
                let (node_name, node_namespace, topic_type) = unsafe {
                    (
                        CStr::from_ptr(info.node_name)
                            .to_string_lossy()
                            .into_owned(),
                        CStr::from_ptr(info.node_namespace)
                            .to_string_lossy()
                            .into_owned(),
                        CStr::from_ptr(info.topic_type)
                            .to_string_lossy()
                            .into_owned(),
                    )
                };
                TopicEndpointInfo {
                    node_name,
                    node_namespace,
                    topic_type,
                }
            })
            .collect();

        Ok(topic_endpoint_infos_vec)
    }
}

/// Converts a rmw_names_and_types_t object to a HashMap.
fn convert_names_and_types(
    rcl_names_and_types: rmw_names_and_types_t,
) -> HashMap<String, Vec<String>> {
    let mut names_and_types: TopicNamesAndTypes = HashMap::new();

    // SAFETY: Safe if the rcl_names_and_types arg has been initialized by the caller
    let name_slice = unsafe {
        rcl_from_raw_parts(
            rcl_names_and_types.names.data,
            rcl_names_and_types.names.size,
        )
    };

    for (idx, name) in name_slice.iter().enumerate() {
        // SAFETY: The slice contains valid C string pointers if it was populated by the caller
        let name: String = unsafe {
            let cstr = CStr::from_ptr(*name);
            cstr.to_string_lossy().into_owned()
        };

        // SAFETY: Safe as long as rcl_names_and_types was populated by the caller
        let types: Vec<String> = unsafe {
            let p = rcl_names_and_types.types.add(idx);
            rcl_from_raw_parts((*p).data, (*p).size)
                .iter()
                .map(|s| {
                    let cstr = CStr::from_ptr(*s);
                    cstr.to_string_lossy().into_owned()
                })
                .collect()
        };

        names_and_types.insert(name, types);
    }

    names_and_types
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::*;

    #[test]
    fn test_graph_empty() {
        // cargo test by default will run all test functions in parallel using
        // as many threads as the underlying system allows. However, the test
        // expectations of test_graph_empty will fail if its detects any other middleware
        // activity while it's running.
        //
        // If we ensure that the Context of test_graph_empty is given a different domain ID
        // from the rest of the tests, then we can ensure that it will not observe any other
        // middleware activity, and its expectations can pass (as long as the user is not
        // running any other ROS executables on their system).
        //
        // By default we will assign 99 to the domain ID of test_graph_empty's Context.
        // However, if the ROS_DOMAIN_ID environment variable was set to 99 by the user,
        // then the rest of the tests will be using that value. So here we are detecting
        // that situation and setting the domain ID of test_graph_empty's Context to 98
        // in that situation.
        //
        // 99 and 98 are just chosen as arbitrary valid domain ID values. There is
        // otherwise nothing special about either value.
        let domain_id: usize = std::env::var("ROS_DOMAIN_ID")
            .ok()
            .and_then(|value| value.parse().ok())
            .map(|value: usize| if value != 99 { 99 } else { 98 })
            .unwrap_or(99);

        let executor = Context::new([], InitOptions::new().with_domain_id(Some(domain_id)))
            .unwrap()
            .create_basic_executor();
        let node_name = "test_publisher_names_and_types";
        let node = executor.create_node(node_name).unwrap();

        let check_rosout = |topics: HashMap<String, Vec<String>>| {
            // rosout shows up in humble and iron, even if the graph is empty
            #[cfg(ros_distro = "humble")]
            {
                assert_eq!(topics.len(), 1);
                assert_eq!(
                    topics.get("/rosout").unwrap().first().unwrap(),
                    "rcl_interfaces/msg/Log"
                );
            }

            // rosout does not automatically show up in jazzy when the graph is empty
            #[cfg(any(ros_distro = "jazzy", ros_distro = "rolling"))]
            {
                assert_eq!(topics.len(), 0);
            }
        };

        let names_and_topics = node
            .get_publisher_names_and_types_by_node(node_name, "")
            .unwrap();
        check_rosout(names_and_topics);

        let num_publishers = node.count_publishers("/test").unwrap();

        assert_eq!(num_publishers, 0);

        let publisher_infos = node.get_publishers_info_by_topic("test").unwrap();

        assert!(publisher_infos.is_empty());

        // Test that the graph has no subscriptions
        let names_and_topics = node
            .get_subscription_names_and_types_by_node(node_name, "")
            .unwrap();

        assert_eq!(names_and_topics.len(), 0);

        let num_subscriptions = node.count_subscriptions("/test").unwrap();

        assert_eq!(num_subscriptions, 0);

        let subscription_infos = node.get_subscriptions_info_by_topic("test").unwrap();

        assert!(subscription_infos.is_empty());

        // Test that the graph only has 6 services (parameter services)
        let names_and_topics = node
            .get_service_names_and_types_by_node(node_name, "")
            .unwrap();

        assert_eq!(names_and_topics.len(), 6);

        let names_and_topics = node.get_service_names_and_types().unwrap();

        assert_eq!(names_and_topics.len(), 6);

        // Test that the graph has no clients
        let names_and_topics = node
            .get_client_names_and_types_by_node(node_name, "")
            .unwrap();

        assert_eq!(names_and_topics.len(), 0);

        let names_and_topics = node.get_topic_names_and_types().unwrap();
        check_rosout(names_and_topics);
    }

    #[test]
    fn test_node_names() {
        let executor = Context::default().create_basic_executor();
        let node_name = "test_node_names";
        let node = executor.create_node(node_name).unwrap();

        let names_and_namespaces = node.get_node_names().unwrap();

        // The tests are executed in parallel, so we might see some other nodes
        // in here. That's why we can't use assert_eq.
        assert!(names_and_namespaces.contains(&NodeNameInfo {
            name: node_name.to_string(),
            namespace: "/".to_string()
        }));
    }

    #[test]
    fn test_node_names_with_enclaves() {
        let executor = Context::default().create_basic_executor();
        let node_name = "test_node_names_with_enclaves";
        let node = executor.create_node(node_name).unwrap();

        let names_and_namespaces = node.get_node_names_with_enclaves().unwrap();

        // The tests are executed in parallel, so we might see some other nodes
        // in here. That's why we can't use assert_eq.
        assert!(names_and_namespaces.contains(&(
            NodeNameInfo {
                name: node_name.to_string(),
                namespace: "/".to_string()
            },
            "/".to_string()
        )));
    }
}

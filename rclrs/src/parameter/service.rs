use std::{
    collections::BTreeSet,
    sync::{Arc, Mutex},
};

use crate::vendor::rcl_interfaces::{msg::rmw::*, srv::rmw::*};
use rosidl_runtime_rs::Sequence;

use super::ParameterMap;
use crate::{
    parameter::{DeclaredValue, ParameterKind, ParameterStorage},
    rmw_request_id_t, Node, RclrsError, Service,
};

// The variables only exist to keep a strong reference to the services and are technically unused.
// What is used is the Weak that is stored in the node, and is upgraded when spinning.
pub struct ParameterService {
    #[allow(dead_code)]
    describe_parameters_service: Arc<Service<DescribeParameters>>,
    #[allow(dead_code)]
    get_parameter_types_service: Arc<Service<GetParameterTypes>>,
    #[allow(dead_code)]
    get_parameters_service: Arc<Service<GetParameters>>,
    #[allow(dead_code)]
    list_parameters_service: Arc<Service<ListParameters>>,
    #[allow(dead_code)]
    set_parameters_service: Arc<Service<SetParameters>>,
    #[allow(dead_code)]
    set_parameters_atomically_service: Arc<Service<SetParametersAtomically>>,
}

fn describe_parameters(
    req: DescribeParameters_Request,
    map: &ParameterMap,
) -> DescribeParameters_Response {
    let descriptors = req
        .names
        .into_iter()
        .map(|name| {
            let name = name.to_cstr().to_str().ok()?;
            let Some(storage) = map.storage.get(name) else {
                return Some(ParameterDescriptor {
                    name: name.into(),
                    ..Default::default()
                });
            };
            let mut descriptor = match storage {
                ParameterStorage::Declared(storage) => {
                    let (integer_range, floating_point_range) =
                        storage.options.ranges.to_descriptor_ranges();
                    ParameterDescriptor {
                        name: name.into(),
                        type_: Default::default(),
                        description: storage.options.description.clone().into(),
                        additional_constraints: storage.options.constraints.clone().into(),
                        dynamic_typing: matches!(storage.kind, ParameterKind::Dynamic),
                        read_only: matches!(storage.value, DeclaredValue::ReadOnly(_)),
                        floating_point_range,
                        integer_range,
                    }
                }
                ParameterStorage::Undeclared(_) => ParameterDescriptor {
                    name: name.into(),
                    dynamic_typing: true,
                    ..Default::default()
                },
            };
            descriptor.type_ = storage.to_parameter_type();
            Some(descriptor)
        })
        .collect::<Option<_>>()
        .unwrap_or_default();
    // TODO(luca) error logging if the service call failed
    DescribeParameters_Response { descriptors }
}

fn get_parameter_types(
    req: GetParameterTypes_Request,
    map: &ParameterMap,
) -> GetParameterTypes_Response {
    let types = req
        .names
        .into_iter()
        .map(|name| {
            let name = name.to_cstr().to_str().ok()?;
            map.storage
                .get(name)
                .map(|s| s.to_parameter_type())
                .or(Some(ParameterType::PARAMETER_NOT_SET))
        })
        .collect::<Option<_>>()
        .unwrap_or_default();
    GetParameterTypes_Response { types }
}

fn get_parameters(req: GetParameters_Request, map: &ParameterMap) -> GetParameters_Response {
    let values = req
        .names
        .into_iter()
        .map(|name| {
            let name = name.to_cstr().to_str().ok()?;
            let Some(storage) = map.storage.get(name) else {
                return Some(ParameterValue::default());
            };
            match storage {
                ParameterStorage::Declared(storage) => match &storage.value {
                    DeclaredValue::Mandatory(v) => Some(v.read().unwrap().clone().into()),
                    DeclaredValue::Optional(v) => Some(
                        v.read()
                            .unwrap()
                            .clone()
                            .map(|v| v.into())
                            .unwrap_or_default(),
                    ),
                    DeclaredValue::ReadOnly(v) => Some(v.clone().into()),
                },
                ParameterStorage::Undeclared(value) => Some(value.clone().into()),
            }
        })
        .collect::<Option<_>>()
        .unwrap_or_default();
    GetParameters_Response { values }
}

fn list_parameters(req: ListParameters_Request, map: &ParameterMap) -> ListParameters_Response {
    let check_parameter_name_depth = |substring: &[std::os::raw::c_char]| {
        if req.depth == ListParameters_Request::DEPTH_RECURSIVE {
            return true;
        }
        u64::try_from(substring.iter().filter(|c| **c == ('.' as _)).count()).unwrap() < req.depth
    };
    let names: Sequence<_> = map
        .storage
        .keys()
        .filter_map(|name| {
            let name: rosidl_runtime_rs::String = name.clone().into();
            if req.prefixes.len() == 0 && check_parameter_name_depth(&name[..]) {
                return Some(name);
            }
            req.prefixes
                .iter()
                .any(|prefix| {
                    if name == *prefix {
                        return true;
                    }
                    let mut prefix = prefix.clone();
                    prefix.extend(".".chars());
                    if name.len() > prefix.len()
                        && name.starts_with(&prefix)
                        && check_parameter_name_depth(&name[prefix.len()..])
                    {
                        return true;
                    }
                    false
                })
                .then_some(name)
        })
        .collect();
    let prefixes: BTreeSet<rosidl_runtime_rs::String> = names
        .iter()
        .filter_map(|name| {
            let name = name.to_string();
            if let Some(pos) = name.rfind('.') {
                return Some(name[0..pos].into());
            }
            None
        })
        .collect();
    ListParameters_Response {
        result: ListParametersResult {
            names,
            prefixes: prefixes.into_iter().collect(),
        },
    }
}

fn set_parameters(req: SetParameters_Request, map: &mut ParameterMap) -> SetParameters_Response {
    let results = req
        .parameters
        .into_iter()
        .map(|param| {
            let Ok(name) = param.name.to_cstr().to_str() else {
                return SetParametersResult {
                    successful: false,
                    reason: "Failed parsing into UTF-8".into(),
                };
            };
            match map.validate_parameter_setting(name, param.value) {
                Ok(value) => {
                    map.store_parameter(name.into(), value);
                    SetParametersResult {
                        successful: true,
                        reason: Default::default(),
                    }
                }
                Err(e) => SetParametersResult {
                    successful: false,
                    reason: e.into(),
                },
            }
        })
        .collect();
    SetParameters_Response { results }
}

fn set_parameters_atomically(
    req: SetParametersAtomically_Request,
    map: &mut ParameterMap,
) -> SetParametersAtomically_Response {
    let results = req
        .parameters
        .into_iter()
        .map(|param| {
            let Ok(name) = param.name.to_cstr().to_str() else {
                return Err("Failed parsing into UTF-8".into());
            };
            let value = map.validate_parameter_setting(name, param.value)?;
            Ok((name.into(), value))
        })
        .collect::<Result<Vec<_>, _>>();
    // Check if there was any error and update parameters accordingly
    let result = match results {
        Ok(results) => {
            for (name, value) in results.into_iter() {
                map.store_parameter(name, value);
            }
            SetParametersResult {
                successful: true,
                reason: Default::default(),
            }
        }
        Err(reason) => SetParametersResult {
            successful: false,
            reason,
        },
    };
    SetParametersAtomically_Response { result }
}

impl ParameterService {
    pub(crate) fn new(
        node: &Node,
        parameter_map: Arc<Mutex<ParameterMap>>,
    ) -> Result<Self, RclrsError> {
        let fqn = node.fully_qualified_name();
        // TODO(luca) make sure it is OK to have an Arc instead of a Weak here and cleanup on
        // destruction is made for the parameter map.
        let map = parameter_map.clone();
        let describe_parameters_service = node.create_service(
            &(fqn.clone() + "/describe_parameters"),
            move |_req_id: &rmw_request_id_t, req: DescribeParameters_Request| {
                let map = map.lock().unwrap();
                describe_parameters(req, &map)
            },
        )?;
        let map = parameter_map.clone();
        let get_parameter_types_service = node.create_service(
            &(fqn.clone() + "/get_parameter_types"),
            move |_req_id: &rmw_request_id_t, req: GetParameterTypes_Request| {
                let map = map.lock().unwrap();
                get_parameter_types(req, &map)
            },
        )?;
        let map = parameter_map.clone();
        let get_parameters_service = node.create_service(
            &(fqn.clone() + "/get_parameters"),
            move |_req_id: &rmw_request_id_t, req: GetParameters_Request| {
                let map = map.lock().unwrap();
                get_parameters(req, &map)
            },
        )?;
        let map = parameter_map.clone();
        let list_parameters_service = node.create_service(
            &(fqn.clone() + "/list_parameters"),
            move |_req_id: &rmw_request_id_t, req: ListParameters_Request| {
                let map = map.lock().unwrap();
                list_parameters(req, &map)
            },
        )?;
        let map = parameter_map.clone();
        let set_parameters_service = node.create_service(
            &(fqn.clone() + "/set_parameters"),
            move |_req_id: &rmw_request_id_t, req: SetParameters_Request| {
                let mut map = map.lock().unwrap();
                set_parameters(req, &mut map)
            },
        )?;
        let set_parameters_atomically_service = node.create_service(
            &(fqn.clone() + "/set_parameters_atomically"),
            move |_req_id: &rmw_request_id_t, req: SetParametersAtomically_Request| {
                let mut map = parameter_map.lock().unwrap();
                set_parameters_atomically(req, &mut map)
            },
        )?;
        Ok(Self {
            describe_parameters_service,
            get_parameter_types_service,
            get_parameters_service,
            list_parameters_service,
            set_parameters_service,
            set_parameters_atomically_service,
        })
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        vendor::rcl_interfaces::{
            msg::rmw::{
                Parameter as RmwParameter, ParameterType, ParameterValue as RmwParameterValue,
            },
            srv::rmw::*,
        },
        Context, MandatoryParameter, Node, NodeBuilder, ParameterRange, ParameterValue, RclrsError,
        ReadOnlyParameter,
    };
    use rosidl_runtime_rs::{seq, Sequence};
    use std::sync::{Arc, RwLock};

    struct TestNode {
        node: Arc<Node>,
        bool_param: MandatoryParameter<bool>,
        _ns_param: MandatoryParameter<i64>,
        _read_only_param: ReadOnlyParameter<f64>,
        dynamic_param: MandatoryParameter<ParameterValue>,
    }

    async fn try_until_timeout<F>(f: F) -> Result<(), ()>
    where
        F: FnOnce() -> bool + Copy,
    {
        let mut retry_count = 0;
        while !f() {
            if retry_count > 50 {
                return Err(());
            }
            tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
            retry_count += 1;
        }
        Ok(())
    }

    fn construct_test_nodes(context: &Context, ns: &str) -> (TestNode, Arc<Node>) {
        let node = NodeBuilder::new(context, "node")
            .namespace(ns)
            .build()
            .unwrap();
        let range = ParameterRange {
            lower: Some(0),
            upper: Some(100),
            step: None,
        };
        let bool_param = node
            .declare_parameter("bool")
            .default(true)
            .description("A boolean value")
            .mandatory()
            .unwrap();
        let _ns_param = node
            .declare_parameter("ns1.ns2.ns3.int")
            .default(42)
            .range(range)
            .constraints("Only the answer")
            .mandatory()
            .unwrap();
        let _read_only_param = node
            .declare_parameter("read_only")
            .default(1.23)
            .read_only()
            .unwrap();
        let dynamic_param = node
            .declare_parameter("dynamic")
            .default(ParameterValue::String("hello".into()))
            .mandatory()
            .unwrap();

        let client = NodeBuilder::new(context, "client")
            .namespace(ns)
            .build()
            .unwrap();

        (
            TestNode {
                node,
                bool_param,
                _ns_param,
                _read_only_param,
                dynamic_param,
            },
            client,
        )
    }

    #[test]
    fn test_parameter_services_names_and_types() -> Result<(), RclrsError> {
        let context = Context::new([]).unwrap();
        let (node, _client) = construct_test_nodes(&context, "names_types");

        std::thread::sleep(std::time::Duration::from_millis(100));

        let names_and_types = node.node.get_service_names_and_types()?;
        let types = names_and_types
            .get("/names_types/node/describe_parameters")
            .unwrap();
        assert!(types.contains(&"rcl_interfaces/srv/DescribeParameters".to_string()));
        let types = names_and_types
            .get("/names_types/node/get_parameters")
            .unwrap();
        assert!(types.contains(&"rcl_interfaces/srv/GetParameters".to_string()));
        let types = names_and_types
            .get("/names_types/node/set_parameters")
            .unwrap();
        assert!(types.contains(&"rcl_interfaces/srv/SetParameters".to_string()));
        let types = names_and_types
            .get("/names_types/node/set_parameters_atomically")
            .unwrap();
        assert!(types.contains(&"rcl_interfaces/srv/SetParametersAtomically".to_string()));
        let types = names_and_types
            .get("/names_types/node/list_parameters")
            .unwrap();
        assert!(types.contains(&"rcl_interfaces/srv/ListParameters".to_string()));
        let types = names_and_types
            .get("/names_types/node/get_parameter_types")
            .unwrap();
        assert!(types.contains(&"rcl_interfaces/srv/GetParameterTypes".to_string()));
        Ok(())
    }

    #[tokio::test]
    async fn test_list_parameters_service() -> Result<(), RclrsError> {
        let context = Context::new([]).unwrap();
        let (node, client) = construct_test_nodes(&context, "list");
        let list_client = client.create_client::<ListParameters>("/list/node/list_parameters")?;

        try_until_timeout(|| list_client.service_is_ready().unwrap())
            .await
            .unwrap();

        let done = Arc::new(RwLock::new(false));

        let inner_done = done.clone();
        let rclrs_spin = tokio::task::spawn(async move {
            try_until_timeout(|| {
                crate::spin_once(node.node.clone(), Some(std::time::Duration::ZERO)).ok();
                crate::spin_once(client.clone(), Some(std::time::Duration::ZERO)).ok();
                *inner_done.read().unwrap()
            })
            .await
            .unwrap();
        });

        let res = tokio::task::spawn(async move {
            // List all parameters
            let request = ListParameters_Request {
                prefixes: seq![],
                depth: 0,
            };
            let client_finished = Arc::new(RwLock::new(false));
            let call_done = client_finished.clone();
            list_client
                .async_send_request_with_callback(
                    &request,
                    move |response: ListParameters_Response| {
                        // use_sim_time + all the manually defined ones
                        *call_done.write().unwrap() = true;
                        let names = response.result.names;
                        assert_eq!(names.len(), 5);
                        // Parameter names are returned in alphabetical order
                        assert_eq!(names[0].to_string(), "bool");
                        assert_eq!(names[1].to_string(), "dynamic");
                        assert_eq!(names[2].to_string(), "ns1.ns2.ns3.int");
                        assert_eq!(names[3].to_string(), "read_only");
                        assert_eq!(names[4].to_string(), "use_sim_time");
                        // Only one prefix
                        assert_eq!(response.result.prefixes.len(), 1);
                        assert_eq!(response.result.prefixes[0].to_string(), "ns1.ns2.ns3");
                    },
                )
                .unwrap();
            try_until_timeout(|| *client_finished.read().unwrap())
                .await
                .unwrap();

            // Limit depth, namespaced parameter is not returned
            let request = ListParameters_Request {
                prefixes: seq![],
                depth: 1,
            };
            let call_done = client_finished.clone();
            *call_done.write().unwrap() = false;
            list_client
                .async_send_request_with_callback(
                    &request,
                    move |response: ListParameters_Response| {
                        *call_done.write().unwrap() = true;
                        let names = response.result.names;
                        assert_eq!(names.len(), 4);
                        assert!(names.iter().all(|n| n.to_string() != "ns1.ns2.ns3.int"));
                        assert_eq!(response.result.prefixes.len(), 0);
                    },
                )
                .unwrap();
            try_until_timeout(|| *client_finished.read().unwrap())
                .await
                .unwrap();

            // Filter by prefix, just return the requested one with the right prefix
            let request = ListParameters_Request {
                prefixes: seq!["ns1.ns2".into()],
                depth: 0,
            };
            let call_done = client_finished.clone();
            *call_done.write().unwrap() = false;
            list_client
                .async_send_request_with_callback(
                    &request,
                    move |response: ListParameters_Response| {
                        *call_done.write().unwrap() = true;
                        let names = response.result.names;
                        assert_eq!(names.len(), 1);
                        assert_eq!(names[0].to_string(), "ns1.ns2.ns3.int");
                        assert_eq!(response.result.prefixes.len(), 1);
                        assert_eq!(response.result.prefixes[0].to_string(), "ns1.ns2.ns3");
                    },
                )
                .unwrap();
            try_until_timeout(|| *client_finished.read().unwrap())
                .await
                .unwrap();

            // If prefix is equal to names, parameters should be returned
            let request = ListParameters_Request {
                prefixes: seq!["use_sim_time".into(), "bool".into()],
                depth: 0,
            };
            let call_done = client_finished.clone();
            *call_done.write().unwrap() = false;
            list_client
                .async_send_request_with_callback(
                    &request,
                    move |response: ListParameters_Response| {
                        *call_done.write().unwrap() = true;
                        let names = response.result.names;
                        dbg!(&names);
                        assert_eq!(names.len(), 2);
                        assert_eq!(names[0].to_string(), "bool");
                        assert_eq!(names[1].to_string(), "use_sim_time");
                        assert_eq!(response.result.prefixes.len(), 0);
                    },
                )
                .unwrap();
            try_until_timeout(|| *client_finished.read().unwrap())
                .await
                .unwrap();
            *done.write().unwrap() = true;
        });

        res.await.unwrap();
        rclrs_spin.await.unwrap();

        Ok(())
    }

    #[tokio::test]
    async fn test_get_set_parameters_service() -> Result<(), RclrsError> {
        let context = Context::new([]).unwrap();
        let (node, client) = construct_test_nodes(&context, "get_set");
        let get_client = client.create_client::<GetParameters>("/get_set/node/get_parameters")?;
        let set_client = client.create_client::<SetParameters>("/get_set/node/set_parameters")?;
        let set_atomically_client = client
            .create_client::<SetParametersAtomically>("/get_set/node/set_parameters_atomically")?;

        try_until_timeout(|| {
            get_client.service_is_ready().unwrap()
                && set_client.service_is_ready().unwrap()
                && set_atomically_client.service_is_ready().unwrap()
        })
        .await
        .unwrap();

        let done = Arc::new(RwLock::new(false));

        let inner_node = node.node.clone();
        let inner_done = done.clone();
        let rclrs_spin = tokio::task::spawn(async move {
            try_until_timeout(|| {
                crate::spin_once(inner_node.clone(), Some(std::time::Duration::ZERO)).ok();
                crate::spin_once(client.clone(), Some(std::time::Duration::ZERO)).ok();
                *inner_done.read().unwrap()
            })
            .await
            .unwrap();
        });

        let res = tokio::task::spawn(async move {
            // Get an existing parameter
            let request = GetParameters_Request {
                names: seq!["bool".into()],
            };
            let client_finished = Arc::new(RwLock::new(false));
            let call_done = client_finished.clone();
            get_client
                .async_send_request_with_callback(
                    &request,
                    move |response: GetParameters_Response| {
                        *call_done.write().unwrap() = true;
                        assert_eq!(response.values.len(), 1);
                        let param = &response.values[0];
                        assert_eq!(param.type_, ParameterType::PARAMETER_BOOL);
                        assert!(param.bool_value);
                    },
                )
                .unwrap();
            try_until_timeout(|| *client_finished.read().unwrap())
                .await
                .unwrap();

            // Getting both existing and non existing parameters, missing one should return
            // PARAMETER_NOT_SET
            let request = GetParameters_Request {
                names: seq!["bool".into(), "non_existing".into()],
            };
            let client_finished = Arc::new(RwLock::new(false));
            let call_done = client_finished.clone();
            get_client
                .async_send_request_with_callback(
                    &request,
                    move |response: GetParameters_Response| {
                        *call_done.write().unwrap() = true;
                        assert_eq!(response.values.len(), 2);
                        let param = &response.values[0];
                        assert_eq!(param.type_, ParameterType::PARAMETER_BOOL);
                        assert!(param.bool_value);
                        assert_eq!(response.values[1].type_, ParameterType::PARAMETER_NOT_SET);
                    },
                )
                .unwrap();
            try_until_timeout(|| *client_finished.read().unwrap())
                .await
                .unwrap();

            // Set a mix of existing, non existing, dynamic and out of range parameters
            let bool_parameter = RmwParameter {
                name: "bool".into(),
                value: RmwParameterValue {
                    type_: ParameterType::PARAMETER_BOOL,
                    bool_value: false,
                    ..Default::default()
                },
            };
            let bool_parameter_mismatched = RmwParameter {
                name: "bool".into(),
                value: RmwParameterValue {
                    type_: ParameterType::PARAMETER_INTEGER,
                    integer_value: 42,
                    ..Default::default()
                },
            };
            let read_only_parameter = RmwParameter {
                name: "read_only".into(),
                value: RmwParameterValue {
                    type_: ParameterType::PARAMETER_DOUBLE,
                    double_value: 3.45,
                    ..Default::default()
                },
            };
            let dynamic_parameter = RmwParameter {
                name: "dynamic".into(),
                value: RmwParameterValue {
                    type_: ParameterType::PARAMETER_BOOL,
                    bool_value: true,
                    ..Default::default()
                },
            };
            let out_of_range_parameter = RmwParameter {
                name: "ns1.ns2.ns3.int".into(),
                value: RmwParameterValue {
                    type_: ParameterType::PARAMETER_INTEGER,
                    integer_value: 1000,
                    ..Default::default()
                },
            };
            let invalid_parameter_type = RmwParameter {
                name: "dynamic".into(),
                value: RmwParameterValue {
                    type_: 200,
                    integer_value: 1000,
                    ..Default::default()
                },
            };
            let undeclared_bool = RmwParameter {
                name: "undeclared_bool".into(),
                value: RmwParameterValue {
                    type_: ParameterType::PARAMETER_BOOL,
                    bool_value: true,
                    ..Default::default()
                },
            };
            let request = SetParameters_Request {
                parameters: seq![
                    bool_parameter.clone(),
                    read_only_parameter.clone(),
                    bool_parameter_mismatched,
                    dynamic_parameter,
                    out_of_range_parameter,
                    invalid_parameter_type,
                    undeclared_bool.clone()
                ],
            };
            let client_finished = Arc::new(RwLock::new(false));
            let call_done = client_finished.clone();
            // Parameter is assigned a default of true at declaration time
            assert!(node.bool_param.get());
            set_client
                .async_send_request_with_callback(
                    &request,
                    move |response: SetParameters_Response| {
                        *call_done.write().unwrap() = true;
                        assert_eq!(response.results.len(), 7);
                        // Setting a bool value set for a bool parameter
                        assert!(response.results[0].successful);
                        // Value was set to false, node parameter get should reflect this
                        assert!(!node.bool_param.get());
                        // Setting a parameter to the wrong type
                        assert!(!response.results[1].successful);
                        // Setting a read only parameter
                        assert!(!response.results[2].successful);
                        // Setting a dynamic parameter to a new type
                        assert!(response.results[3].successful);
                        assert_eq!(node.dynamic_param.get(), ParameterValue::Bool(true));
                        // Setting a value out of range
                        assert!(!response.results[4].successful);
                        // Setting an invalid type
                        assert!(!response.results[5].successful);
                        // Setting an undeclared parameter, without allowing undeclared parameters
                        assert!(!response.results[6].successful);
                    },
                )
                .unwrap();
            try_until_timeout(|| *client_finished.read().unwrap())
                .await
                .unwrap();

            // Set the node to use undeclared parameters and try to set one
            node.node.use_undeclared_parameters();
            let request = SetParameters_Request {
                parameters: seq![undeclared_bool],
            };
            let client_finished = Arc::new(RwLock::new(false));
            let call_done = client_finished.clone();
            set_client
                .async_send_request_with_callback(
                    &request,
                    move |response: SetParameters_Response| {
                        *call_done.write().unwrap() = true;
                        assert_eq!(response.results.len(), 1);
                        // Setting the undeclared parameter is now allowed
                        assert!(response.results[0].successful);
                        assert_eq!(
                            node.node.use_undeclared_parameters().get("undeclared_bool"),
                            Some(ParameterValue::Bool(true))
                        );
                    },
                )
                .unwrap();
            try_until_timeout(|| *client_finished.read().unwrap())
                .await
                .unwrap();

            // With set_parameters_atomically, if one fails all should fail
            let request = SetParametersAtomically_Request {
                parameters: seq![bool_parameter, read_only_parameter],
            };
            let client_finished = Arc::new(RwLock::new(false));
            let call_done = client_finished.clone();
            set_atomically_client
                .async_send_request_with_callback(
                    &request,
                    move |response: SetParametersAtomically_Response| {
                        *call_done.write().unwrap() = true;
                        assert!(!response.result.successful);
                    },
                )
                .unwrap();
            try_until_timeout(|| *client_finished.read().unwrap())
                .await
                .unwrap();
            *done.write().unwrap() = true;
        });

        res.await.unwrap();
        rclrs_spin.await.unwrap();

        Ok(())
    }

    #[tokio::test]
    async fn test_describe_get_types_parameters_service() -> Result<(), RclrsError> {
        let context = Context::new([]).unwrap();
        let (node, client) = construct_test_nodes(&context, "describe");
        let describe_client =
            client.create_client::<DescribeParameters>("/describe/node/describe_parameters")?;
        let get_types_client =
            client.create_client::<GetParameterTypes>("/describe/node/get_parameter_types")?;

        try_until_timeout(|| {
            describe_client.service_is_ready().unwrap()
                && get_types_client.service_is_ready().unwrap()
        })
        .await
        .unwrap();

        let done = Arc::new(RwLock::new(false));

        let inner_done = done.clone();
        let inner_node = node.node.clone();
        let rclrs_spin = tokio::task::spawn(async move {
            try_until_timeout(|| {
                crate::spin_once(inner_node.clone(), Some(std::time::Duration::ZERO)).ok();
                crate::spin_once(client.clone(), Some(std::time::Duration::ZERO)).ok();
                *inner_done.read().unwrap()
            })
            .await
            .unwrap();
        });

        let res = tokio::task::spawn(async move {
            // Describe all parameters
            let request = DescribeParameters_Request {
                names: seq![
                    "bool".into(),
                    "ns1.ns2.ns3.int".into(),
                    "read_only".into(),
                    "dynamic".into()
                ],
            };
            let client_finished = Arc::new(RwLock::new(false));
            let call_done = client_finished.clone();
            describe_client
                .async_send_request_with_callback(
                    &request,
                    move |response: DescribeParameters_Response| {
                        *call_done.write().unwrap() = true;
                        let desc = response.descriptors;
                        assert_eq!(desc.len(), 4);
                        // Descriptors are returned in the requested order
                        assert_eq!(desc[0].name.to_string(), "bool");
                        assert_eq!(desc[0].type_, ParameterType::PARAMETER_BOOL);
                        assert_eq!(desc[0].description.to_string(), "A boolean value");
                        assert!(!desc[0].read_only);
                        assert!(!desc[0].dynamic_typing);
                        assert_eq!(desc[1].name.to_string(), "ns1.ns2.ns3.int");
                        assert_eq!(desc[1].type_, ParameterType::PARAMETER_INTEGER);
                        assert_eq!(desc[1].integer_range.len(), 1);
                        assert_eq!(desc[1].integer_range[0].from_value, 0);
                        assert_eq!(desc[1].integer_range[0].to_value, 100);
                        assert_eq!(desc[1].integer_range[0].step, 0);
                        assert!(!desc[1].read_only);
                        assert!(!desc[1].dynamic_typing);
                        assert_eq!(
                            desc[1].additional_constraints.to_string(),
                            "Only the answer"
                        );
                        assert_eq!(desc[2].name.to_string(), "read_only");
                        assert_eq!(desc[2].type_, ParameterType::PARAMETER_DOUBLE);
                        assert!(desc[2].read_only);
                        assert!(!desc[2].dynamic_typing);
                        assert_eq!(desc[3].name.to_string(), "dynamic");
                        assert_eq!(desc[3].type_, ParameterType::PARAMETER_STRING);
                        assert!(desc[3].dynamic_typing);
                        assert!(!desc[3].read_only);
                    },
                )
                .unwrap();
            try_until_timeout(|| *client_finished.read().unwrap())
                .await
                .unwrap();

            // If a describe parameters request is sent with a non existing parameter, an empty
            // response should be returned
            let request = DescribeParameters_Request {
                names: seq!["bool".into(), "non_existing".into()],
            };
            let client_finished = Arc::new(RwLock::new(false));
            let call_done = client_finished.clone();
            describe_client
                .async_send_request_with_callback(
                    &request,
                    move |response: DescribeParameters_Response| {
                        *call_done.write().unwrap() = true;
                        assert_eq!(response.descriptors[0].name.to_string(), "bool");
                        assert_eq!(response.descriptors[0].type_, ParameterType::PARAMETER_BOOL);
                        assert_eq!(response.descriptors.len(), 2);
                        assert_eq!(response.descriptors[1].name.to_string(), "non_existing");
                        assert_eq!(
                            response.descriptors[1].type_,
                            ParameterType::PARAMETER_NOT_SET
                        );
                    },
                )
                .unwrap();
            try_until_timeout(|| *client_finished.read().unwrap())
                .await
                .unwrap();

            // Get all parameter types, including a non existing one that will be NOT_SET
            let request = GetParameterTypes_Request {
                names: seq![
                    "bool".into(),
                    "ns1.ns2.ns3.int".into(),
                    "read_only".into(),
                    "dynamic".into(),
                    "non_existing".into()
                ],
            };
            let client_finished = Arc::new(RwLock::new(false));
            let call_done = client_finished.clone();
            get_types_client
                .async_send_request_with_callback(
                    &request,
                    move |response: GetParameterTypes_Response| {
                        *call_done.write().unwrap() = true;
                        assert_eq!(response.types.len(), 5);
                        // Types are returned in the requested order
                        assert_eq!(response.types[0], ParameterType::PARAMETER_BOOL);
                        assert_eq!(response.types[1], ParameterType::PARAMETER_INTEGER);
                        assert_eq!(response.types[2], ParameterType::PARAMETER_DOUBLE);
                        assert_eq!(response.types[3], ParameterType::PARAMETER_STRING);
                        assert_eq!(response.types[4], ParameterType::PARAMETER_NOT_SET);
                    },
                )
                .unwrap();
            try_until_timeout(|| *client_finished.read().unwrap())
                .await
                .unwrap();

            *done.write().unwrap() = true;
        });

        res.await.unwrap();
        rclrs_spin.await.unwrap();

        Ok(())
    }
}

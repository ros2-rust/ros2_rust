use rcl_interfaces::msg::rmw::{
    Parameter as RmwParameter, ParameterType, ParameterValue as RmwParameterValue,
};
use rcl_interfaces::srv::rmw::*;
use rclrs::{
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
            rclrs::spin_once(node.node.clone(), Some(std::time::Duration::ZERO)).ok();
            rclrs::spin_once(client.clone(), Some(std::time::Duration::ZERO)).ok();
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
            .async_send_request_with_callback(&request, move |response: ListParameters_Response| {
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
            })
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
            .async_send_request_with_callback(&request, move |response: ListParameters_Response| {
                *call_done.write().unwrap() = true;
                let names = response.result.names;
                assert_eq!(names.len(), 4);
                assert!(names.iter().all(|n| n.to_string() != "ns1.ns2.ns3.int"));
                assert_eq!(response.result.prefixes.len(), 0);
            })
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
            .async_send_request_with_callback(&request, move |response: ListParameters_Response| {
                *call_done.write().unwrap() = true;
                let names = response.result.names;
                assert_eq!(names.len(), 1);
                assert_eq!(names[0].to_string(), "ns1.ns2.ns3.int");
                assert_eq!(response.result.prefixes.len(), 1);
                assert_eq!(response.result.prefixes[0].to_string(), "ns1.ns2.ns3");
            })
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
            .async_send_request_with_callback(&request, move |response: ListParameters_Response| {
                *call_done.write().unwrap() = true;
                let names = response.result.names;
                dbg!(&names);
                assert_eq!(names.len(), 2);
                assert_eq!(names[0].to_string(), "bool");
                assert_eq!(names[1].to_string(), "use_sim_time");
                assert_eq!(response.result.prefixes.len(), 0);
            })
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
            rclrs::spin_once(inner_node.clone(), Some(std::time::Duration::ZERO)).ok();
            rclrs::spin_once(client.clone(), Some(std::time::Duration::ZERO)).ok();
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
            .async_send_request_with_callback(&request, move |response: GetParameters_Response| {
                *call_done.write().unwrap() = true;
                assert_eq!(response.values.len(), 1);
                let param = &response.values[0];
                assert_eq!(param.type_, ParameterType::PARAMETER_BOOL);
                assert!(param.bool_value);
            })
            .unwrap();
        try_until_timeout(|| *client_finished.read().unwrap())
            .await
            .unwrap();

        // Getting both existing and non existing parameters should fail
        let request = GetParameters_Request {
            names: seq!["bool".into(), "non_existing".into()],
        };
        let client_finished = Arc::new(RwLock::new(false));
        let call_done = client_finished.clone();
        get_client
            .async_send_request_with_callback(&request, move |response: GetParameters_Response| {
                *call_done.write().unwrap() = true;
                assert_eq!(response.values.len(), 0);
            })
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
            .async_send_request_with_callback(&request, move |response: SetParameters_Response| {
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
            })
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
            .async_send_request_with_callback(&request, move |response: SetParameters_Response| {
                *call_done.write().unwrap() = true;
                assert_eq!(response.results.len(), 1);
                // Setting the undeclared parameter is now allowed
                assert!(response.results[0].successful);
                assert_eq!(
                    node.node.use_undeclared_parameters().get("undeclared_bool"),
                    Some(ParameterValue::Bool(true))
                );
            })
            .unwrap();
        try_until_timeout(|| *client_finished.read().unwrap())
            .await
            .unwrap();

        // Now getting an undefined parameter should work
        let request = GetParameters_Request {
            names: seq!["non_existing".into()],
        };
        let client_finished = Arc::new(RwLock::new(false));
        let call_done = client_finished.clone();
        get_client
            .async_send_request_with_callback(&request, move |response: GetParameters_Response| {
                *call_done.write().unwrap() = true;
                assert_eq!(response.values.len(), 1);
                assert_eq!(response.values[0].type_, ParameterType::PARAMETER_NOT_SET);
            })
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
        describe_client.service_is_ready().unwrap() && get_types_client.service_is_ready().unwrap()
    })
    .await
    .unwrap();

    let done = Arc::new(RwLock::new(false));

    let inner_done = done.clone();
    let inner_node = node.node.clone();
    let rclrs_spin = tokio::task::spawn(async move {
        try_until_timeout(|| {
            rclrs::spin_once(inner_node.clone(), Some(std::time::Duration::ZERO)).ok();
            rclrs::spin_once(client.clone(), Some(std::time::Duration::ZERO)).ok();
            *inner_done.read().unwrap()
        })
        .await
        .unwrap();
    });

    let res = tokio::task::spawn(async move {
        // Desctibe all parameters
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
                    assert_eq!(response.descriptors.len(), 0);
                },
            )
            .unwrap();
        try_until_timeout(|| *client_finished.read().unwrap())
            .await
            .unwrap();

        // Get all parameter types
        let request = GetParameterTypes_Request {
            names: seq![
                "bool".into(),
                "ns1.ns2.ns3.int".into(),
                "read_only".into(),
                "dynamic".into()
            ],
        };
        let client_finished = Arc::new(RwLock::new(false));
        let call_done = client_finished.clone();
        get_types_client
            .async_send_request_with_callback(
                &request,
                move |response: GetParameterTypes_Response| {
                    *call_done.write().unwrap() = true;
                    assert_eq!(response.types.len(), 4);
                    // Types are returned in the requested order
                    assert_eq!(response.types[0], ParameterType::PARAMETER_BOOL);
                    assert_eq!(response.types[1], ParameterType::PARAMETER_INTEGER);
                    assert_eq!(response.types[2], ParameterType::PARAMETER_DOUBLE);
                    assert_eq!(response.types[3], ParameterType::PARAMETER_STRING);
                },
            )
            .unwrap();
        try_until_timeout(|| *client_finished.read().unwrap())
            .await
            .unwrap();

        // Once undeclared parameters are allowed, non existing should return a not set type
        node.node.use_undeclared_parameters();
        let request = GetParameterTypes_Request {
            names: seq!["non_existing".into()],
        };
        let client_finished = Arc::new(RwLock::new(false));
        let call_done = client_finished.clone();
        get_types_client
            .async_send_request_with_callback(
                &request,
                move |response: GetParameterTypes_Response| {
                    *call_done.write().unwrap() = true;
                    assert_eq!(response.types.len(), 1);
                    assert_eq!(response.types[0], ParameterType::PARAMETER_NOT_SET);
                },
            )
            .unwrap();
        try_until_timeout(|| *client_finished.read().unwrap())
            .await
            .unwrap();

        // Once undeclared parameters are allowed, non existing should return a not set descriptor
        let request = DescribeParameters_Request {
            names: seq!["non_existing".into()],
        };
        let client_finished = Arc::new(RwLock::new(false));
        let call_done = client_finished.clone();
        describe_client
            .async_send_request_with_callback(
                &request,
                move |response: DescribeParameters_Response| {
                    *call_done.write().unwrap() = true;
                    assert_eq!(response.descriptors.len(), 1);
                    assert_eq!(response.descriptors[0].name.to_string(), "non_existing");
                    assert_eq!(
                        response.descriptors[0].type_,
                        ParameterType::PARAMETER_NOT_SET
                    );
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

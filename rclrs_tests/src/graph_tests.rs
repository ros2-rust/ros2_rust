use rclrs::{
    Context, Node, NodeBuilder, RclrsError, TopicEndpointInfo, TopicNamesAndTypes,
    QOS_PROFILE_SYSTEM_DEFAULT,
};
use test_msgs::{msg, srv};

struct TestGraph {
    node1: Node,
    node2: Node,
}

fn construct_test_graph(namespace: &str) -> Result<TestGraph, RclrsError> {
    let context = Context::new([])?;
    Ok(TestGraph {
        node1: NodeBuilder::new(&context, "graph_test_node_1")
            .namespace(namespace)
            .build()?,
        node2: NodeBuilder::new(&context, "graph_test_node_2")
            .namespace(namespace)
            .build()?,
    })
}

#[test]
fn test_publishers() -> Result<(), RclrsError> {
    let namespace = "/test_publishers_graph";
    let graph = construct_test_graph(namespace)?;

    let node_1_empty_publisher = graph
        .node1
        .create_publisher::<msg::Empty>("graph_test_topic_1", QOS_PROFILE_SYSTEM_DEFAULT)?;
    let topic1 = node_1_empty_publisher.topic_name();
    let node_1_basic_types_publisher = graph
        .node1
        .create_publisher::<msg::BasicTypes>("graph_test_topic_2", QOS_PROFILE_SYSTEM_DEFAULT)?;
    let topic2 = node_1_basic_types_publisher.topic_name();
    let node_2_default_publisher = graph
        .node2
        .create_publisher::<msg::Defaults>("graph_test_topic_3", QOS_PROFILE_SYSTEM_DEFAULT)?;
    let topic3 = node_2_default_publisher.topic_name();

    std::thread::sleep(std::time::Duration::from_millis(100));

    // Test count_publishers()
    assert_eq!(graph.node1.count_publishers(&topic1)?, 1);
    assert_eq!(graph.node1.count_publishers(&topic2)?, 1);
    assert_eq!(graph.node1.count_publishers(&topic3)?, 1);

    // Test get_publisher_names_and_types_by_node()
    let node_1_publisher_names_and_types = graph
        .node1
        .get_publisher_names_and_types_by_node(&graph.node1.name(), namespace)?;

    let types = node_1_publisher_names_and_types.get(&topic1).unwrap();
    assert!(types.contains(&"test_msgs/msg/Empty".to_string()));

    let types = node_1_publisher_names_and_types.get(&topic2).unwrap();
    assert!(types.contains(&"test_msgs/msg/BasicTypes".to_string()));

    let node_2_publisher_names_and_types = graph
        .node1
        .get_publisher_names_and_types_by_node(&graph.node2.name(), namespace)?;

    let types = node_2_publisher_names_and_types.get(&topic3).unwrap();
    assert!(types.contains(&"test_msgs/msg/Defaults".to_string()));

    // Test get_publishers_info_by_topic()
    let expected_publishers_info = vec![TopicEndpointInfo {
        node_name: String::from("graph_test_node_1"),
        node_namespace: String::from(namespace),
        topic_type: String::from("test_msgs/msg/Empty"),
    }];
    assert_eq!(
        graph.node1.get_publishers_info_by_topic(&topic1)?,
        expected_publishers_info
    );
    assert_eq!(
        graph.node2.get_publishers_info_by_topic(&topic1)?,
        expected_publishers_info
    );

    Ok(())
}

#[test]
fn test_subscriptions() -> Result<(), RclrsError> {
    let namespace = "/test_subscriptions_graph";
    let mut graph = construct_test_graph(namespace)?;

    let node_2_empty_subscription = graph.node2.create_subscription::<msg::Empty, _>(
        "graph_test_topic_1",
        QOS_PROFILE_SYSTEM_DEFAULT,
        |_msg: msg::Empty| {},
    )?;
    let topic1 = node_2_empty_subscription.topic_name();
    let node_2_basic_types_subscription = graph.node2.create_subscription::<msg::BasicTypes, _>(
        "graph_test_topic_2",
        QOS_PROFILE_SYSTEM_DEFAULT,
        |_msg: msg::BasicTypes| {},
    )?;
    let topic2 = node_2_basic_types_subscription.topic_name();
    let node_1_defaults_subscription = graph.node1.create_subscription::<msg::Defaults, _>(
        "graph_test_topic_3",
        QOS_PROFILE_SYSTEM_DEFAULT,
        |_msg: msg::Defaults| {},
    )?;
    let topic3 = node_1_defaults_subscription.topic_name();

    std::thread::sleep(std::time::Duration::from_millis(100));

    // Test count_subscriptions()
    assert_eq!(graph.node2.count_subscriptions(&topic1)?, 1);
    assert_eq!(graph.node2.count_subscriptions(&topic2)?, 1);

    // Test get_subscription_names_and_types_by_node()
    let node_1_subscription_names_and_types = graph
        .node1
        .get_subscription_names_and_types_by_node(&graph.node1.name(), namespace)?;

    let types = node_1_subscription_names_and_types.get(&topic3).unwrap();
    assert!(types.contains(&"test_msgs/msg/Defaults".to_string()));

    let node_2_subscription_names_and_types = graph
        .node2
        .get_subscription_names_and_types_by_node(&graph.node2.name(), namespace)?;

    let types = node_2_subscription_names_and_types.get(&topic1).unwrap();
    assert!(types.contains(&"test_msgs/msg/Empty".to_string()));

    let types = node_2_subscription_names_and_types.get(&topic2).unwrap();
    assert!(types.contains(&"test_msgs/msg/BasicTypes".to_string()));

    // Test get_subscriptions_info_by_topic()
    let expected_subscriptions_info = vec![TopicEndpointInfo {
        node_name: String::from("graph_test_node_2"),
        node_namespace: String::from(namespace),
        topic_type: String::from("test_msgs/msg/Empty"),
    }];
    assert_eq!(
        graph.node1.get_subscriptions_info_by_topic(&topic1)?,
        expected_subscriptions_info
    );
    assert_eq!(
        graph.node2.get_subscriptions_info_by_topic(&topic1)?,
        expected_subscriptions_info
    );
    Ok(())
}

#[test]
fn test_topic_names_and_types() -> Result<(), RclrsError> {
    let mut graph = construct_test_graph("test_topics_graph")?;

    let _node_1_defaults_subscription = graph.node1.create_subscription::<msg::Defaults, _>(
        "graph_test_topic_3",
        QOS_PROFILE_SYSTEM_DEFAULT,
        |_msg: msg::Defaults| {},
    )?;
    let _node_2_empty_subscription = graph.node2.create_subscription::<msg::Empty, _>(
        "graph_test_topic_1",
        QOS_PROFILE_SYSTEM_DEFAULT,
        |_msg: msg::Empty| {},
    )?;
    let _node_2_basic_types_subscription = graph.node2.create_subscription::<msg::BasicTypes, _>(
        "graph_test_topic_2",
        QOS_PROFILE_SYSTEM_DEFAULT,
        |_msg: msg::BasicTypes| {},
    )?;

    std::thread::sleep(std::time::Duration::from_millis(100));

    let topic_names_and_types = graph.node1.get_topic_names_and_types()?;

    let types = topic_names_and_types
        .get("/test_topics_graph/graph_test_topic_1")
        .unwrap();
    assert!(types.contains(&"test_msgs/msg/Empty".to_string()));
    let types = topic_names_and_types
        .get("/test_topics_graph/graph_test_topic_2")
        .unwrap();
    assert!(types.contains(&"test_msgs/msg/BasicTypes".to_string()));

    let types = topic_names_and_types
        .get("/test_topics_graph/graph_test_topic_3")
        .unwrap();
    assert!(types.contains(&"test_msgs/msg/Defaults".to_string()));

    Ok(())
}

#[test]
fn test_services() -> Result<(), RclrsError> {
    let namespace = "/test_services_graph";
    let mut graph = construct_test_graph(namespace)?;
    let check_names_and_types = |names_and_types: TopicNamesAndTypes| {
        let types = names_and_types
            .get("/test_services_graph/graph_test_topic_4")
            .unwrap();
        assert!(types.contains(&"test_msgs/srv/Empty".to_string()));
    };

    let _node_1_empty_service =
        graph
            .node1
            .create_service::<srv::Empty, _>("graph_test_topic_4", |_, _| srv::Empty_Response {
                structure_needs_at_least_one_member: 0,
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

#[test]
fn test_clients() -> Result<(), RclrsError> {
    let namespace = "/test_clients_graph";
    let mut graph = construct_test_graph(namespace)?;
    let _node_2_empty_client = graph
        .node2
        .create_client::<srv::Empty>("graph_test_topic_4")?;

    std::thread::sleep(std::time::Duration::from_millis(200));

    let client_names_and_types = graph
        .node2
        .get_client_names_and_types_by_node(&graph.node2.name(), &graph.node2.namespace())?;
    let types = client_names_and_types
        .get("/test_clients_graph/graph_test_topic_4")
        .unwrap();

    assert!(types.contains(&"test_msgs/srv/Empty".to_string()));

    Ok(())
}

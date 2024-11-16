use crate::{Context, Node, RclrsError, NodeOptions};

pub(crate) struct TestGraph {
    pub node1: Node,
    pub node2: Node,
}

pub(crate) fn construct_test_graph(namespace: &str) -> Result<TestGraph, RclrsError> {
    let executor = Context::default().create_basic_executor();
    Ok(TestGraph {
        node1: executor.create_node(
            NodeOptions::new("graph_test_node_1")
            .namespace(namespace)
        )?,
        node2: executor.create_node(
            NodeOptions::new("graph_test_node_2")
            .namespace(namespace)
        )?,
    })
}

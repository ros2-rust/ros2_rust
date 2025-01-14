use crate::*;
use std::sync::Arc;

pub(crate) struct TestGraph {
    pub node1: Arc<Node>,
    pub node2: Arc<Node>,
}

pub(crate) fn construct_test_graph(namespace: &str) -> Result<TestGraph, RclrsError> {
    let executor = Context::default().create_basic_executor();
    Ok(TestGraph {
        node1: executor.create_node("graph_test_node_1".namespace(namespace))?,
        node2: executor.create_node("graph_test_node_2".namespace(namespace))?,
    })
}

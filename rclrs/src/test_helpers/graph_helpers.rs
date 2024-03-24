use crate::{Context, Node, NodeBuilder, RclrsError};
use std::sync::Arc;

pub(crate) struct TestGraph {
    pub node1: Arc<Node>,
    pub node2: Arc<Node>,
}

pub(crate) fn construct_test_graph(namespace: &str) -> Result<TestGraph, RclrsError> {
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

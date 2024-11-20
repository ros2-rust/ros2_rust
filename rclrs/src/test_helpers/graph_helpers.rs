use crate::{Context, Node, NodeBuilder, RclrsError};

pub(crate) struct TestGraph {
    pub node1: Node,
    pub node2: Node,
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

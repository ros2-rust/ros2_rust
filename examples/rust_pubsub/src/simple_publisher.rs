use rclrs::{create_node, Context, Node, Publisher, RclrsError, QOS_PROFILE_DEFAULT};
/// Creates a SimplePublisherNode, initializes a node and publisher, and provides
/// methods to publish a simple "Hello World" message on a loop in separate threads.

/// Imports the Arc type from std::sync, used for thread-safe reference counting pointers,
/// and the StringMsg message type from std_msgs for publishing string messages.
use std::{sync::Arc, thread, time::Duration};
use std_msgs::msg::String as StringMsg;
// / SimplePublisherNode struct contains node and publisher members.
// / Used to initialize a ROS 2 node and publisher, and publish messages.
struct SimplePublisherNode {
    node: Arc<Node>,
    _publisher: Arc<Publisher<StringMsg>>,
}
/// The `new` function takes a context and returns a Result containing the
/// initialized SimplePublisherNode or an error. It creates a node with the
/// given name and creates a publisher on the "publish_hello" topic.
///
/// The SimplePublisherNode contains the node and publisher members.
impl SimplePublisherNode {
    /// Creates a new SimplePublisherNode by initializing a node and publisher.
    ///
    /// This function takes a context and returns a Result containing the
    /// initialized SimplePublisherNode or an error. It creates a node with the
    /// given name and creates a publisher on the "publish_hello" topic.
    ///
    /// The SimplePublisherNode contains the node and publisher members.
    fn new(context: &Context) -> Result<Self, RclrsError> {
        let node = create_node(context, "simple_publisher").unwrap();
        let _publisher = node
            .create_publisher("publish_hello", QOS_PROFILE_DEFAULT)
            .unwrap();
        Ok(Self { node, _publisher })
    }

    /// Publishes a "Hello World" message on the publisher.
    ///
    /// Creates a StringMsg with "Hello World" as the data, publishes it on
    /// the `_publisher`, and returns a Result. This allows regularly publishing
    /// a simple message on a loop.
    fn publish_data(&self, increment: i32) -> Result<i32, RclrsError> {
        let msg: StringMsg = StringMsg {
            data: format!("Hello World {}", increment),
        };
        self._publisher.publish(msg).unwrap();
        Ok(increment + 1_i32)
    }
}

/// The main function initializes a ROS 2 context, node and publisher,
/// spawns a thread to publish messages repeatedly, and spins the node
/// to receive callbacks.
///
/// It creates a context, initializes a SimplePublisherNode which creates
/// a node and publisher, clones the publisher to pass to the thread,  
/// spawns a thread to publish "Hello World" messages repeatedly, and
/// calls spin() on the node to receive callbacks. This allows publishing
/// messages asynchronously while spinning the node.
fn main() -> Result<(), RclrsError> {
    let context = Context::new(std::env::args()).unwrap();
    let publisher = Arc::new(SimplePublisherNode::new(&context).unwrap());
    let publisher_other_thread = Arc::clone(&publisher);
    let mut count: i32 = 0;
    thread::spawn(move || loop {
        thread::sleep(Duration::from_millis(1000));
        count = publisher_other_thread.publish_data(count).unwrap();
    });
    rclrs::spin(publisher.node.clone())
}

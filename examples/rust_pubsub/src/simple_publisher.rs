use rclrs::{create_node, Context, Node, Publisher, RclrsError, QOS_PROFILE_DEFAULT};
use std::{sync::Arc, thread, time::Duration};
use std_msgs::msg::String as StringMsg;
struct SimplePublisherNode {
    node: Arc<Node>,
    _publisher: Arc<Publisher<StringMsg>>,
}
impl SimplePublisherNode {
    fn new(context: &Context) -> Result<Self, RclrsError> {
        let node = create_node(context, "simple_publisher").unwrap();
        let _publisher = node
            .create_publisher("publish_hello", QOS_PROFILE_DEFAULT)
            .unwrap();
        Ok(Self { node, _publisher })
    }

    fn publish_data(&self, increment: i32) -> Result<i32, RclrsError> {
        let msg: StringMsg = StringMsg {
            data: format!("Hello World {}", increment),
        };
        self._publisher.publish(msg).unwrap();
        Ok(increment + 1_i32)
    }
}

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

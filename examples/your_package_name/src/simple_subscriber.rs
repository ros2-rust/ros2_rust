use rclrs::{create_node, Context, Node, RclrsError, Subscription, QOS_PROFILE_DEFAULT};
use std::{
    env, iter,
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};
use std_msgs::msg::String as StringMsg;
/// A simple ROS2 subscriber node that receives and prints "hello" messages.
///
/// This node creates a subscription to the "publish_hello" topic and prints the
/// received messages to the console. It runs the subscription in a separate
/// thread, while the main thread calls `rclrs::spin()` to keep the node running.
pub struct SimpleSubscriptionNode {
    node: Arc<Node>,
    _subscriber: Arc<Subscription<StringMsg>>,
    data: Arc<Mutex<Option<StringMsg>>>,
}
/// Implements a simple ROS2 subscriber node that receives and prints "hello" messages.
/// The `new` function creates the node and the subscription, and returns a `SimpleSubscriptionNode`
/// instance. The `data_callback` function can be used to access the latest received message.
impl SimpleSubscriptionNode {
    fn new(context: &Context) -> Result<Self, RclrsError> {
        let node = create_node(context, "simple_subscription").unwrap();
        let data: Arc<Mutex<Option<StringMsg>>> = Arc::new(Mutex::new(None));
        let data_mut: Arc<Mutex<Option<StringMsg>>> = Arc::clone(&data);
        let _subscriber = node
            .create_subscription::<StringMsg, _>(
                "publish_hello",
                QOS_PROFILE_DEFAULT,
                move |msg: StringMsg| {
                    *data_mut.lock().unwrap() = Some(msg);
                },
            )
            .unwrap();
        Ok(Self {
            node,
            _subscriber,
            data,
        })
    }
    fn data_callback(&self) -> Result<(), RclrsError> {
        if let Some(data) = self.data.lock().unwrap().as_ref() {
            println!("{}", data.data);
        } else {
            println!("No message available yet.");
        }
        Ok(())
    }
}
/// The `main` function creates a new ROS2 context, a `SimpleSubscriptionNode` instance, and starts a separate thread to periodically call the `data_callback` method on the subscription. The main thread then calls `rclrs::spin()` to keep the node running and receive messages.
///
/// The separate thread is used to ensure that the `data_callback` method is called regularly, even if the main thread is blocked in `rclrs::spin()`. This allows the subscriber to continuously process and print the received "hello" messages.
fn main() -> Result<(), RclrsError> {
    let context = Context::new(env::args()).unwrap();
    let subscription = Arc::new(SimpleSubscriptionNode::new(&context).unwrap());
    let subscription_other_thread = Arc::clone(&subscription);
    thread::spawn(move || -> () {
        iter::repeat(()).for_each(|()| {
            thread::sleep(Duration::from_millis(1000));
            subscription_other_thread.data_callback().unwrap()
        });
    });
    rclrs::spin(subscription.node.clone())
}

use rclrs::*;
use std::sync::Arc;

use anyhow::{Error, Result};

struct MinimalSubscriberNode {
    #[allow(unused)]
    subscription: WorkerSubscription<example_interfaces::msg::String, SubscriptionData>,
}

struct SubscriptionData {
    node: Node,
    num_messages: usize,
}

impl MinimalSubscriberNode {
    pub fn new(executor: &Executor, name: &str, topic: &str) -> Result<Self, RclrsError> {
        let node = executor.create_node(name)?;

        let worker = node.create_worker::<SubscriptionData>(SubscriptionData {
            node: Arc::clone(&node),
            num_messages: 0,
        });

        let subscription = worker.create_subscription(
            topic,
            |data: &mut SubscriptionData, msg: example_interfaces::msg::String| {
                data.num_messages += 1;
                println!("[{}] I heard: '{}'", data.node.name(), msg.data);
                println!(
                    "[{}] (Got {} messages so far)",
                    data.node.name(),
                    data.num_messages,
                );
            },
        )?;

        Ok(MinimalSubscriberNode { subscription })
    }
}

fn main() -> Result<(), Error> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let publisher_node = executor.create_node("minimal_publisher")?;

    let _subscriber_node_one =
        MinimalSubscriberNode::new(&executor, "minimal_subscriber_one", "topic")?;
    let _subscriber_node_two =
        MinimalSubscriberNode::new(&executor, "minimal_subscriber_two", "topic")?;

    let publisher = publisher_node.create_publisher::<example_interfaces::msg::String>("topic")?;

    // TODO(@mxgrey): Replace this with a timer once we have the Timer feature
    // merged in.
    std::thread::spawn(move || -> Result<(), rclrs::RclrsError> {
        let mut message = example_interfaces::msg::String::default();
        let mut publish_count: u32 = 1;
        loop {
            message.data = format!("Hello, world! {}", publish_count);
            println!("Publishing: [{}]", message.data);
            publisher.publish(&message)?;
            publish_count += 1;
            std::thread::sleep(std::time::Duration::from_millis(500));
        }
    });

    executor.spin(rclrs::SpinOptions::default()).first_error()?;
    Ok(())
}

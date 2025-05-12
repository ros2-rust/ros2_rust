use rclrs::*;
use std::sync::{
    atomic::{AtomicU32, Ordering},
    Arc, Mutex,
};

use anyhow::{Error, Result};

struct MinimalSubscriber {
    num_messages: AtomicU32,
    node: Node,
    subscription: Mutex<Option<Subscription<std_msgs::msg::String>>>,
}

impl MinimalSubscriber {
    pub fn new(executor: &Executor, name: &str, topic: &str) -> Result<Arc<Self>, RclrsError> {
        let node = executor.create_node(name)?;
        let minimal_subscriber = Arc::new(MinimalSubscriber {
            num_messages: 0.into(),
            node,
            subscription: None.into(),
        });

        let minimal_subscriber_aux = Arc::clone(&minimal_subscriber);
        let subscription = minimal_subscriber
            .node
            .create_subscription::<std_msgs::msg::String, _>(
                topic,
                move |msg: std_msgs::msg::String| {
                    minimal_subscriber_aux.callback(msg);
                },
            )?;
        *minimal_subscriber.subscription.lock().unwrap() = Some(subscription);
        Ok(minimal_subscriber)
    }

    fn callback(&self, msg: std_msgs::msg::String) {
        self.num_messages.fetch_add(1, Ordering::SeqCst);
        println!("[{}] I heard: '{}'", self.node.name(), msg.data);
        println!(
            "[{}] (Got {} messages so far)",
            self.node.name(),
            self.num_messages.load(Ordering::SeqCst)
        );
    }
}

fn main() -> Result<(), Error> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let publisher_node = executor.create_node("minimal_publisher")?;

    let _subscriber_node_one =
        MinimalSubscriber::new(&executor, "minimal_subscriber_one", "topic")?;
    let _subscriber_node_two =
        MinimalSubscriber::new(&executor, "minimal_subscriber_two", "topic")?;

    let publisher = publisher_node.create_publisher::<std_msgs::msg::String>("topic")?;

    std::thread::spawn(move || -> Result<(), RclrsError> {
        let mut message = std_msgs::msg::String::default();
        let mut publish_count: u32 = 1;
        loop {
            message.data = format!("Hello, world! {}", publish_count);
            println!("Publishing: [{}]", message.data);
            publisher.publish(&message)?;
            publish_count += 1;
            std::thread::sleep(std::time::Duration::from_millis(500));
        }
    });

    executor
        .spin(SpinOptions::default())
        .first_error()
        .map_err(|err| err.into())
}

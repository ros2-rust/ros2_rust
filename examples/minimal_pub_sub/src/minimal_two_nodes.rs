use std::{
    env,
    sync::{
        atomic::{AtomicU32, Ordering},
        Arc, Mutex,
    },
};

use anyhow::{Error, Result};

struct MinimalSubscriber {
    num_messages: AtomicU32,
    node: Arc<rclrs::Node>,
    subscription: Mutex<Option<Arc<rclrs::Subscription<std_msgs::msg::String>>>>,
}

impl MinimalSubscriber {
    pub fn new(name: &str, topic: &str) -> Result<Arc<Self>, rclrs::RclrsError> {
        let context = rclrs::Context::new(env::args())?;
        let node = rclrs::create_node(&context, name)?;
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
                rclrs::QOS_PROFILE_DEFAULT,
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
    let publisher_context = rclrs::Context::new(env::args())?;
    let publisher_node = rclrs::create_node(&publisher_context, "minimal_publisher")?;

    let subscriber_node_one = MinimalSubscriber::new("minimal_subscriber_one", "topic")?;
    let subscriber_node_two = MinimalSubscriber::new("minimal_subscriber_two", "topic")?;

    let publisher = publisher_node
        .create_publisher::<std_msgs::msg::String>("topic", rclrs::QOS_PROFILE_DEFAULT)?;

    std::thread::spawn(move || -> Result<(), rclrs::RclrsError> {
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

    let executor = rclrs::SingleThreadedExecutor::new();

    executor.add_node(&publisher_node)?;
    executor.add_node(&subscriber_node_one.node)?;
    executor.add_node(&subscriber_node_two.node)?;

    executor.spin().map_err(|err| err.into())
}

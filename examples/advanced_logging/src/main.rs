use anyhow::{Error, Result};
use rclrs::*;
use std::time::Duration;

fn main() -> Result<(), Error> {
    let context = Context::default_from_env()?;
    let executor = context.create_basic_executor();

    let node = executor.create_node("advanced_logger")?;

    let publisher = node.create_publisher::<std_msgs::msg::String>("topic", QOS_PROFILE_DEFAULT)?;

    let mut message = std_msgs::msg::String::default();

    let mut publish_count: u32 = 1;

    while context.ok() {
        message.data = format!("Hello, world! {}", publish_count);
        // log_fatal!(&node.name(), "Simple message from {}", node.name());
        log!(
            node.info().skip_first(),
            "Publish every message but the first one: [{}]",
            message.data
        );
        log!(
            node.warn().throttle(Duration::from_millis(3000)),
            "Publish with 3s throttling: [{}]",
            message.data
        );
        log!(
            node.error().only_if(publish_count % 10 == 0),
            "Publishing every 10 messages: [{}]",
            message.data
        );
        publisher.publish(&message)?;
        publish_count += 1;
        std::thread::sleep(std::time::Duration::from_millis(500));
    }
    Ok(())
}

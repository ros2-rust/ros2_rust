use std::env;

use anyhow::{Error, Result};

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;

    let node = rclrs::create_node(&context, "minimal_publisher")?;

    let publisher =
        node.create_publisher::<std_msgs::msg::String>("topic", rclrs::QOS_PROFILE_DEFAULT)?;

    let mut message = std_msgs::msg::String::default();

    let mut publish_count: u32 = 1;

    while context.ok() {
        message.data = format!("Hello, world! {}", publish_count);
        println!("Publishing: [{}]", message.data);
        publisher.publish(&message)?;
        publish_count += 1;
        std::thread::sleep(std::time::Duration::from_millis(500));
    }
    Ok(())
}

use anyhow::{Error, Result};
use std::env;

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;

    let node = rclrs::create_node(&context, "minimal_publisher")?;

    let publisher =
        node.create_publisher::<std_msgs::msg::rmw::UInt32>("topic", rclrs::QOS_PROFILE_DEFAULT)?;

    let mut publish_count: u32 = 1;

    while context.ok() {
        let mut message = publisher.borrow_loaned_message()?;
        message.data = publish_count;
        println!("Publishing: {}", message.data);
        message.publish()?;
        publish_count += 1;
        std::thread::sleep(std::time::Duration::from_millis(500));
    }
    Ok(())
}

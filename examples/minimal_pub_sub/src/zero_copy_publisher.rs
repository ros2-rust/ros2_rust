use anyhow::{Error, Result};
use rclrs::*;

fn main() -> Result<(), Error> {
    let context = Context::default_from_env()?;
    let executor = context.create_basic_executor();

    let node = executor.create_node("minimal_publisher")?;

    let publisher =
        node.create_publisher::<std_msgs::msg::rmw::UInt32>("topic", QOS_PROFILE_DEFAULT)?;

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

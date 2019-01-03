use rclrs::{self, RclResult};
use std::{thread, time};
use std_msgs;

fn main() -> RclResult {
    let ros = rclrs::Context::default();
    let node = ros.new_node("rclrs_publisher")?;
    let publisher = node.advertise::<std_msgs::msg::String>("greetings")?;

    while ros.ok() {
        let mut message = std_msgs::msg::String::default();
        message.data = "Hello, world!".to_string();
        publisher.publish(&message)?;
        thread::sleep(time::Duration::from_secs(1));
    }

    Ok(())
}

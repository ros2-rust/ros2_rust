use rclrs::{self, RclResult};
use std_msgs;
use sensor_msgs;

fn main() -> RclResult {
    let ros = rclrs::Context::default();
    let node = ros.new_node("rclrs_publisher")?;
    let subscription = node.subscribe::<std_msgs::msg::String>("greetings")?;
    let subscription2 = node.subscribe::<std_msgs::msg::Header>("header")?;

    while ros.ok() {
        let mut message = std_msgs::msg::String::default();
        if subscription.take(&mut message).is_ok() {
            println!("{}", message.data);
        }
        let mut message = std_msgs::msg::Header::default();
        if subscription2.take(&mut message).is_ok() {
            println!("{:?}", message);
        }
    }

    Ok(())
}

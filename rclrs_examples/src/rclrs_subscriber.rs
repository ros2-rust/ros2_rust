use rclrs::{self, RclResult};
use std::{thread, time};
use std_msgs;
use sensor_msgs;

fn main() -> RclResult {
    let ros = rclrs::Context::default();
    let node = ros.new_node("rclrs_publisher")?;
    let subscription = node.subscribe::<std_msgs::msg::String>("greetings")?;
    let subscription2 = node.subscribe::<std_msgs::msg::Header>("header")?;
    let subscription3 = node.subscribe::<sensor_msgs::msg::JointState>("joint_state")?;

    while ros.ok() {
        let mut message = std_msgs::msg::String::default();
        if subscription.take(&mut message).is_ok() {
            println!("{:?}", message);
        }
        let mut message = std_msgs::msg::Header::default();
        if subscription2.take(&mut message).is_ok() {
            println!("{:?}", message);
        }
        let mut message = sensor_msgs::msg::JointState::default();
        if subscription3.take(&mut message).is_ok() {
            println!("{:?}", message);
        }
        thread::sleep(time::Duration::from_millis(100));
    }

    Ok(())
}

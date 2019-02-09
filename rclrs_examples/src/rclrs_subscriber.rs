use rclrs;
use std_msgs;

fn topic_callback(msg: &std_msgs::msg::String) {
    println!("I heard: '{}'", msg.data);
}

fn main() -> rclrs::RclResult {
    let context = rclrs::Context::default();

    let mut node = context.create_node("minimal_subscriber")?;

    let publisher = node.create_subscription::<std_msgs::msg::String>(
        "topic",
        rclrs::QOS_PROFILE_DEFAULT,
        topic_callback,
    )?;

    node.spin()
}
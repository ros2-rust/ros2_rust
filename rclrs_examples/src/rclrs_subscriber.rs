use rclrs;
use std_msgs;

fn main() -> rclrs::RclResult {
    let context = rclrs::Context::default();

    let mut node = context.create_node("minimal_subscriber")?;

    let mut num_messages: usize = 0;

    let _subscription = node.create_subscription::<std_msgs::msg::String, _>(
        "topic",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: &std_msgs::msg::String| {
            println!("I heard: '{}'", msg.data);
            println!("(Got {} messages so far)", num_messages);
            num_messages += 1;
        },
    )?;

    rclrs::spin(&node)
}

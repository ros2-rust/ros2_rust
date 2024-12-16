use anyhow::{Error, Result};
use rclrs::RclrsErrorFilter;

fn main() -> Result<(), Error> {
    let context = rclrs::Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    let node = executor.create_node("minimal_subscriber")?;

    let mut num_messages: usize = 0;

    let _subscription = node.create_subscription::<std_msgs::msg::String, _>(
        "topic",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: std_msgs::msg::String| {
            num_messages += 1;
            println!("I heard: '{}'", msg.data);
            println!("(Got {} messages so far)", num_messages);
        },
    )?;

    executor
        .spin(rclrs::SpinOptions::default())
        .first_error()
        .map_err(|err| err.into())
}

use anyhow::{Error, Result};
use rclrs::*;

fn main() -> Result<(), Error> {
    let mut executor = Context::default_from_env()?.create_basic_executor();

    let node = executor.create_node("minimal_subscriber")?;

    let mut num_messages: usize = 0;

    let _subscription = node.create_subscription::<std_msgs::msg::UInt32, _>(
        "topic",
        move |msg: rclrs::ReadOnlyLoanedMessage<'_, std_msgs::msg::UInt32>| {
            num_messages += 1;
            println!("I heard: '{}'", msg.data);
            println!("(Got {} messages so far)", num_messages);
        },
    )?;

    executor
        .spin(SpinOptions::default())
        .first_error()
        .map_err(|err| err.into())
}

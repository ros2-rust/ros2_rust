use anyhow::{Error, Result};
use std::sync::Mutex;
use rclrs::ReadOnlyLoanedMessage;

fn main() -> Result<(), Error> {
    let mut executor = rclrs::Context::default_from_env()?.create_basic_executor();

    let node = executor.create_node("minimal_subscriber")?;

    let num_messages = Mutex::new(0usize);

    let _subscription = node.create_subscription::<std_msgs::msg::UInt32, _>(
        "topic",
        move |msg: ReadOnlyLoanedMessage<std_msgs::msg::UInt32>| {
            let mut num = num_messages.lock().unwrap();
            *num += 1;
            println!("I heard: '{}'", msg.data);
            println!("(Got {} messages so far)", *num);
        },
    )?;

    executor.spin(rclrs::SpinOptions::default())?;
    Ok(())
}

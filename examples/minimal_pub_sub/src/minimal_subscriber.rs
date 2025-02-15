use anyhow::{Error, Result};
use std::sync::Mutex;
use rclrs::{Context, SpinOptions};

fn main() -> Result<(), Error> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    let node = executor.create_node("minimal_subscriber")?;

    let num_messages = Mutex::new(0usize);
    let _subscription = node.create_subscription::<std_msgs::msg::String, _>(
        "topic",
        move |msg: std_msgs::msg::String| {
            let mut num = num_messages.lock().unwrap();
            *num += 1;
            println!("I heard: '{}'", msg.data);
            println!("(Got {} messages so far)", num);
        },
    )?;

    println!("Waiting for messages...");
    executor.spin(SpinOptions::default())?;
    Ok(())
}

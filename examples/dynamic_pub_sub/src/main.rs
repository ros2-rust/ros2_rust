use anyhow::{Error, Result};
use rclrs::*;

fn main() -> Result<(), Error> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    let node = executor.create_node("dynamic_subscriber")?;

    let worker = node.create_worker::<usize>(0);
    let _subscription = worker.create_dynamic_subscription(
        "rclrs_example_msgs/msg/VariousTypes".try_into()?,
        "topic",
        move |num_messages: &mut usize, msg, _msg_info| {
            *num_messages += 1;
            println!("#{} | I heard: '{:#?}'", *num_messages, msg.structure());
        },
    )?;

    println!("Waiting for messages...");
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

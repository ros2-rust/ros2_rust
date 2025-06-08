use anyhow::{Error, Result};
use rclrs::*;

fn main() -> Result<(), Error> {
    let mut executor = Context::default_from_env()?.create_basic_executor();

    let node = executor.create_node("minimal_subscriber")?;

    let worker = node.create_worker::<usize>(0);
    let _subscription = worker.create_subscription::<example_interfaces::msg::UInt32, _>(
        "topic",
        move |num_messages: &mut usize,
              msg: ReadOnlyLoanedMessage<example_interfaces::msg::UInt32>| {
            *num_messages += 1;
            println!("I heard: '{}'", msg.data);
            println!("(Got {} messages so far)", *num_messages);
        },
    )?;

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

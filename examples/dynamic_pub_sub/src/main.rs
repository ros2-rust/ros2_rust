use anyhow::{Error, Result};
use std::env;

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;

    let mut node = rclrs::create_node(&context, "dynamic_subscriber")?;

    let mut num_messages: usize = 0;

    let _subscription = node.create_dynamic_subscription(
        "topic",
        "rclrs_example_msgs/msg/VariousTypes",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg| {
            num_messages += 1;
            println!("I heard: '{:#?}'", msg.structure());
        },
    )?;

    rclrs::spin(&node).map_err(|err| err.into())
}

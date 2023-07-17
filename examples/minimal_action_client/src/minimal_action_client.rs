use std::env;

use anyhow::{Error, Result};

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;

    let mut node = rclrs::create_node(&context, "minimal_client")?;

    let _client =
        node.create_action_client::<example_interfaces::action::Fibonacci>("fibonacci")?;

    Ok(())
}

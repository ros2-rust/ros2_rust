use anyhow::{Error, Result};
use rclrs::*;

fn main() -> Result<(), Error> {
    let mut executor = Context::default_from_env()?.create_basic_executor();

    let mut node = executor.create_node("minimal_action_client")?;

    let _client = node.create_action_client::<example_interfaces::action::Fibonacci>("fibonacci")?;

    executor
        .spin(SpinOptions::default())
        .first_error()
        .map_err(|err| err.into())
}

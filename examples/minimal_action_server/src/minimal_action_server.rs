use anyhow::{Error, Result};
use rclrs::*;

fn main() -> Result<(), Error> {
    let mut executor = Context::default_from_env()?.create_basic_executor();

    let _node = executor.create_node("minimal_action_server")?;

    executor
        .spin(SpinOptions::default())
        .first_error()
        .map_err(|err| err.into())
}

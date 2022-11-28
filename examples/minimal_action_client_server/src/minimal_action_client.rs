use std::env;

use anyhow::{Error, Result};

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;

    let _node = rclrs::create_node(&context, "minimal_action_client")?;

    Ok(())
}

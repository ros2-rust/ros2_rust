use anyhow::{Error, Result};
use std::env;

#[tokio::main]
async fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;

    let mut node = context.create_node("minimal_client")?;

    let client = node.create_client::<example_interfaces::srv::AddTwoInts>("add_two_ints")?;

    println!("Starting client");

    client.wait_for_service(500)?;

    let request = example_interfaces::srv::AddTwoInts_Request { a: 41, b: 1 };

    let future = client.call_async(&request);

    println!("Waiting for response");

    let rclrs_spin = tokio::task::spawn_blocking(move || rclrs::spin(&node));

    let response = future.await?;
    println!(
        "Result of {} + {} is: {}",
        request.a, request.b, response.sum
    );

    rclrs_spin.await.ok();
    Ok(())
}

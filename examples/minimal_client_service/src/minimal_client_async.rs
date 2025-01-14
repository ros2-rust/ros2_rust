use anyhow::{Error, Result};
use rclrs::*;

#[tokio::main]
async fn main() -> Result<(), Error> {
    let mut executor = Context::default_from_env()?.create_basic_executor();

    let node = executor.create_node("minimal_client")?;

    let client = node.create_client::<example_interfaces::srv::AddTwoInts>("add_two_ints")?;

    println!("Starting client");

    while !client.service_is_ready()? {
        std::thread::sleep(std::time::Duration::from_millis(10));
    }

    let request = example_interfaces::srv::AddTwoInts_Request { a: 41, b: 1 };

    let future = client.call_async(&request);

    println!("Waiting for response");

    let rclrs_spin = tokio::task::spawn_blocking(move || executor.spin(SpinOptions::default()));

    let response = future.await?;
    println!(
        "Result of {} + {} is: {}",
        request.a, request.b, response.sum
    );

    rclrs_spin.await.ok();
    Ok(())
}

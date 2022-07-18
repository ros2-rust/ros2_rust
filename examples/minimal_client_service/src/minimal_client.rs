use anyhow::{Error, Result};
use std::env;

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;

    let mut node = rclrs::create_node(&context, "minimal_client")?;

    let client = node.create_client::<example_interfaces::srv::AddTwoInts>("add_two_ints")?;

    let request = example_interfaces::srv::AddTwoInts_Request { a: 41, b: 1 };

    println!("Starting client");

    std::thread::sleep(std::time::Duration::from_millis(500));

    client.async_send_request_with_callback(
        &request,
        move |response: example_interfaces::srv::AddTwoInts_Response| {
            println!(
                "Result of {} + {} is: {}",
                request.a, request.b, response.sum
            );
        },
    )?;

    std::thread::sleep(std::time::Duration::from_millis(500));

    println!("Waiting for response");
    rclrs::spin(&node).map_err(|err| err.into())
}

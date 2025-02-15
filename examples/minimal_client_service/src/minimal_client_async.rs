use anyhow::{Error, Result};
use rclrs::{Context, SpinOptions};

fn main() -> Result<(), Error> {
    let mut executor = Context::default_from_env()?.create_basic_executor();

    let node = executor.create_node("minimal_client")?;

    let client = node.create_client::<example_interfaces::srv::AddTwoInts>("add_two_ints")?;

    println!("Starting client");

    while !client.service_is_ready()? {
        std::thread::sleep(std::time::Duration::from_millis(10));
    }

    let request = example_interfaces::srv::AddTwoInts_Request { a: 41, b: 1 };

    let promise = client.call_then(
        &request,
        move |response: example_interfaces::srv::AddTwoInts_Response| {
            println!(
                "Result of {} + {} is: {}",
                request.a, request.b, response.sum,
            );
        }
    ).unwrap();

    println!("Waiting for response");
    executor.spin(SpinOptions::new().until_promise_resolved(promise))?;
    Ok(())
}

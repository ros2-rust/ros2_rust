use anyhow::{Error, Result};
use example_interfaces::srv::*;
use rclrs::*;

fn main() -> Result<(), Error> {
    let mut executor = Context::default_from_env()?.create_basic_executor();

    let node = executor.create_node("minimal_client")?;

    let client = node.create_client::<AddTwoInts>("add_two_ints")?;

    let promise = executor.commands().run(async move {
        println!("Waiting for service...");
        client.notify_on_service_ready().await.unwrap();

        let request = AddTwoInts_Request { a: 41, b: 1 };

        println!("Waiting for response");
        let response: AddTwoInts_Response = client.call(&request).unwrap().await.unwrap();

        println!(
            "Result of {} + {} is: {}",
            request.a, request.b, response.sum,
        );
    });

    executor
        .spin(SpinOptions::new().until_promise_resolved(promise))
        .first_error()?;
    Ok(())
}

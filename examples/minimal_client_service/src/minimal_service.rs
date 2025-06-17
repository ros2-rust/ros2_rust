use anyhow::{Error, Result};
use example_interfaces::srv::*;
use rclrs::*;

fn handle_service(request: AddTwoInts_Request, info: ServiceInfo) -> AddTwoInts_Response {
    let timestamp = info
        .received_timestamp
        .map(|t| format!(" at [{t:?}]"))
        .unwrap_or(String::new());

    println!("request{timestamp}: {} + {}", request.a, request.b);
    AddTwoInts_Response {
        sum: request.a + request.b,
    }
}

fn main() -> Result<(), Error> {
    let mut executor = Context::default_from_env()?.create_basic_executor();

    let node = executor.create_node("minimal_service")?;

    let _server = node.create_service::<AddTwoInts, _>("add_two_ints", handle_service)?;

    println!("Starting server");
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

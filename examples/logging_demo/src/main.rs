use rclrs::*;
use std::time::Duration;

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("logging_demo")?;

    let _subscription = node.clone().create_subscription(
        "logging_demo",
        move |msg: example_interfaces::msg::String| {
            let data = msg.data;

            // You can apply modifiers such as .once() to node.logger()
            // to dictate how the logging behaves.
            log!(node.logger().once(), "First message: {data}",);

            log!(node.logger().skip_first(), "Subsequent message: {data}",);

            // You can chain multiple modifiers together.
            log_warn!(
                node.logger().skip_first().throttle(Duration::from_secs(5)),
                "Throttled message: {data}",
            );
        },
    )?;

    // Any &str can be used as the logger name and have
    // logging modifiers applied to it.
    log_info!(
        "notice".once(),
        "Ready to begin logging example_interfaces/msg/String messages published to 'logging_demo'.",
    );
    log_warn!(
        "help",
        "Try running\n \
        $ ros2 topic pub logging_demo example_interfaces/msg/String \"data: message\"",
    );
    executor.spin(SpinOptions::default()).first_error()
}

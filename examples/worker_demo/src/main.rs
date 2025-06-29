use rclrs::*;
use std::time::Duration;

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("worker_demo")?;

    let publisher = node.create_publisher("output_topic")?;
    let worker = node.create_worker(String::new());

    let _subscription = worker.create_subscription(
        "input_topic",
        move |data: &mut String, msg: example_interfaces::msg::String| {
            *data = msg.data;
        },
    )?;

    let _timer =
        worker.create_timer_repeating(Duration::from_secs(1), move |data: &mut String| {
            let msg = example_interfaces::msg::String { data: data.clone() };

            publisher.publish(msg).ok();
        })?;

    println!(
        "Beginning repeater... \n >> \
        Publish a std_msg::msg::String to \"input_topic\" and we will periodically republish it to \"output_topic\".\n\n\
        To see this in action run the following commands in two different terminals:\n \
        $ ros2 topic echo output_topic\n \
        $ ros2 topic pub input_topic std_msgs/msg/String \"{{data: Hello}}\""
    );
    executor.spin(SpinOptions::default());

    Ok(())
}

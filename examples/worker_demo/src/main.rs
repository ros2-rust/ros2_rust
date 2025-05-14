use rclrs::*;
use std::sync::Arc;

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

    // // Use this timer-based implementation when timers are available instead
    // // of using std::thread::spawn.
    // let _timer = worker.create_timer_repeating(
    //     Duration::from_secs(1),
    //     move |data: &mut String| {
    //         let msg = example_interfaces::msg::String {
    //             data: data.clone()
    //         };

    //         publisher.publish(msg).ok();
    //     }
    // )?;

    std::thread::spawn(move || loop {
        std::thread::sleep(std::time::Duration::from_secs(1));
        let publisher = Arc::clone(&publisher);
        let _ = worker.run(move |data: &mut String| {
            let msg = example_interfaces::msg::String { data: data.clone() };
            publisher.publish(msg).unwrap();
        });
    });

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

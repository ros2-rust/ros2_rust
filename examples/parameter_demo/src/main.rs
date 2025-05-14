use rclrs::*;
use std::sync::Arc;

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("parameter_demo")?;

    let greeting: MandatoryParameter<Arc<str>> = node
        .declare_parameter("greeting")
        .default("Hello".into())
        .mandatory()?;

    let _subscription =
        node.create_subscription("greet", move |msg: example_interfaces::msg::String| {
            println!("{}, {}", greeting.get(), msg.data);
        })?;

    println!(
        "Ready to provide a greeting. \
        \n\nTo see a greeting, try running\n \
        $ ros2 topic pub greet example_interfaces/msg/String \"data: Alice\"\
        \n\nTo change the kind of greeting, try running\n \
        $ ros2 param set parameter_demo greeting \"Guten tag\"\n"
    );
    executor.spin(SpinOptions::default()).first_error()
}

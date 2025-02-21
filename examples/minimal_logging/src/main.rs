use rclrs::{create_node, log, Context, RclrsError, ToLogParams, QOS_PROFILE_DEFAULT};
use std::{env, time::Duration};
use std_msgs::msg::String as StringMsg;
fn main() -> Result<(), RclrsError> {
    let context = Context::new(env::args())?;

    let node = create_node(&context, "minimal_logger")?;

    let publisher = node.create_publisher("topic", QOS_PROFILE_DEFAULT)?;

    let mut message = StringMsg::default();

    let mut publish_count: u32 = 1;

    while context.ok() {
        message.data = format!("Hello, world! {}", publish_count);
        log!(node.info(), "Publishing: [{}]", message.data);
        publisher.publish(&message)?;
        publish_count += 1;
        std::thread::sleep(std::time::Duration::from_millis(500));
    }
    Ok(())
}

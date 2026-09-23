//! Receive host pixels through the existing Image API.
use rclrs::{Context, CreateBasicExecutor, RclrsErrorFilter, SpinOptions};
use ros_env::sensor_msgs::msg::Image;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("cpu_image_subscriber")?;
    let _subscription = node.create_subscription::<Image, _>("image", |image: Image| {
        println!("received CPU Image pixels={:?}", image.data);
    })?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

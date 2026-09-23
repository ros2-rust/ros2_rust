//! Publish CPU-backed images using the opt-in buffer representation.
use std::time::Duration;

use rclrs::{Context, CreateBasicExecutor, Publisher, RclrsErrorFilter, SpinOptions};
use ros_env::sensor_msgs::msg::buffer::Image;

fn publish_image(publisher: &Publisher<Image>) -> Result<(), Box<dyn std::error::Error>> {
    let image = Image {
        height: 1,
        width: 3,
        encoding: "mono8".into(),
        step: 3,
        data: vec![10, 20, 30].into(),
        ..Default::default()
    };
    publisher.publish(image)?;
    println!("published CPU image");
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("buffer_image_publisher")?;
    let publisher = node.create_publisher::<Image>("image")?;
    let _timer = node.create_timer_repeating(Duration::from_secs(1), move || {
        if let Err(error) = publish_image(&publisher) {
            eprintln!("publish failed: {error}");
        }
    })?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

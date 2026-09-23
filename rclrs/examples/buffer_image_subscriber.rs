//! Receive images as CPU-backed buffers.
use std::borrow::Cow;

use rclrs::{Context, CreateBasicExecutor, RclrsErrorFilter, SpinOptions, SubscriptionOptions};
use ros_env::sensor_msgs::msg::buffer::Image;

fn print_image(image: Image) -> Result<(), Box<dyn std::error::Error>> {
    let expected = u64::from(image.height) * u64::from(image.step);
    if image.data.len() as u64 != expected {
        return Err("image payload length does not match height * step".into());
    }
    let backend = image.data.backend_name()?;
    let pixels = match image.data.as_slice() {
        Some(pixels) => Cow::Borrowed(pixels),
        None => Cow::Owned(image.data.to_vec()?),
    };
    println!("received backend={backend} pixels={pixels:?}");
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("buffer_image_subscriber")?;
    let _subscription = node.create_subscription::<Image, _>(
        SubscriptionOptions::new("image").acceptable_buffer_backends("cpu"),
        |image: Image| {
            if let Err(error) = print_image(image) {
                eprintln!("receive failed: {error}");
            }
        },
    )?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

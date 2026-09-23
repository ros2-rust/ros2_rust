//! Convert a CPU-backed buffer image into the existing CPU representation.
use ros_env::sensor_msgs::msg::{buffer, Image};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let image = buffer::Image {
        height: 1,
        width: 3,
        encoding: "mono8".into(),
        step: 3,
        data: vec![10, 20, 30].into(),
        ..Default::default()
    };
    let cpu: Image = image.try_into_cpu()?;
    println!("pixels: {:?}", cpu.data);
    Ok(())
}

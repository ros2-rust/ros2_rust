extern crate rclrs;
extern crate std_msgs;

fn main() {
    rclrs::init().unwrap();

    let node = rclrs::create_node("minimal_publisher");
    let publisher =
        node.create_publisher::<std_msgs::msg::String>("topic", rclrs::qos::QOS_PROFILE_DEFAULT);

    let mut message: std_msgs::msg::String = Default::default();

    let mut publish_count: u32 = 1;

    while rclrs::ok() {
        message.data = format!("Hello, world! {}", publish_count);
        println!("Publishing: [{}]", message.data);
        publisher.publish(&message).unwrap();
        publish_count += 1;
        std::thread::sleep(std::time::Duration::from_millis(1500));
    }
}

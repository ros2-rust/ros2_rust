use std::env;

use anyhow::{Error, Result};

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;

    let node = rclrs::create_node(&context, "minimal_subscriber")?;
    let param = node.declare_parameter("test").default(42).mandatory().unwrap();
    let p2 = node.declare_parameter("hello").default(1.23).read_only().unwrap();
    let p3 = node.declare_parameter::<bool>("bool").optional().unwrap();
    let p4 = node.declare_parameter::<rclrs::ParameterValue>("dynamic").optional().unwrap();

    let mut num_messages: usize = 0;

    let _subscription = node.create_subscription::<std_msgs::msg::String, _>(
        "topic",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: std_msgs::msg::String| {
            num_messages += 1;
            println!("I heard: '{}'", msg.data);
            println!("(Got {} messages so far)", num_messages);
        },
    )?;

    rclrs::spin(node).map_err(|err| err.into())
}

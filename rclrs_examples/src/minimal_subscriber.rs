use std::env;

use anyhow::{Error, Result};
use cstr_core::CString;
use rclrs;
use std_msgs;

fn main() -> Result<(), Error> {
    let args: Vec<CString> = env::args()
        .filter_map(|arg| CString::new(arg).ok())
        .collect();
    let context = rclrs::Context::default(args);

    let mut node = context.create_node("minimal_subscriber")?;

    let mut num_messages: usize = 0;

    let _subscription = node.create_subscription::<std_msgs::msg::String, _>(
        "topic",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: &std_msgs::msg::String| {
            num_messages += 1;
            println!("I heard: '{}'", msg.data);
            println!("(Got {} messages so far)", num_messages);
        },
    )?;

    rclrs::spin(&node).map_err(|err| err.into())
}

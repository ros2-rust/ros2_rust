use rclrs::{Context, Node, Publisher, Subscription, QOS_PROFILE_DEFAULT};
use std::env;
use std::sync::Arc;

#[allow(unused)]
struct Mimic {
    output_nh: Arc<Node>,
    input_nh: Arc<Node>,
    twist_pub: Arc<Publisher<geometry_msgs::msg::Twist>>,
    pose_sub: Arc<Subscription<turtlesim_rs_msgs::msg::Pose>>,
}

impl Mimic {
    fn new() -> Self {
        let context = Context::new(env::args()).unwrap();
        let output_nh = rclrs::create_node(&context, "output").unwrap();

        let twist_pub = output_nh
            .create_publisher("/output/cmd_vel", QOS_PROFILE_DEFAULT)
            .unwrap();

        let input_nh = rclrs::create_node(&context, "input").unwrap();

        let twist_pub_clone = Arc::clone(&twist_pub);
        let pose_sub = input_nh
            .create_subscription(
                "/input/pose",
                QOS_PROFILE_DEFAULT,
                move |pose_msg: turtlesim_rs_msgs::msg::Pose| {
                    let mut twist_msg = geometry_msgs::msg::Twist::default();
                    twist_msg.linear.x = pose_msg.linear_velocity as f64;
                    twist_msg.angular.z = pose_msg.angular_velocity as f64;
                    twist_pub_clone.publish(twist_msg).unwrap();
                },
            )
            .unwrap();

        Self {
            output_nh,
            input_nh,
            twist_pub,
            pose_sub,
        }
    }
}

fn main() -> Result<(), rclrs::RclrsError> {
    let mimic = Mimic::new();
    rclrs::spin(mimic.input_nh.clone())?;
    Ok(())
}

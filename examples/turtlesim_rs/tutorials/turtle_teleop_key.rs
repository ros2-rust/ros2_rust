use rclrs::{Node, Publisher};
use std::env;
use std::io;
use std::sync::Arc;
use termion::event::Key;
use termion::input::TermRead;
use termion::raw::IntoRawMode;

struct TeleopTurtle {
    _nh: Arc<Node>,
    linear: f64,
    angular: f64,
    l_scale: f64,
    a_scale: f64,
    twist_pub: Arc<Publisher<geometry_msgs::msg::Twist>>,
}

impl TeleopTurtle {
    pub fn new(context: &rclrs::Context) -> Self {
        let _nh = rclrs::create_node(context, "teleop_turtle").unwrap();

        let l_scale_param = _nh
            .declare_parameter("scale_linear")
            .default(2.0)
            .optional()
            .unwrap();

        let a_scale_param = _nh
            .declare_parameter("scale_angular")
            .default(2.0)
            .optional()
            .unwrap();

        let l_scale = l_scale_param.get().unwrap();
        let a_scale = a_scale_param.get().unwrap();

        let twist_pub = _nh
            .create_publisher("/turtle1/cmd_vel", rclrs::QOS_PROFILE_DEFAULT)
            .unwrap();

        Self {
            _nh,
            linear: 0.0,
            angular: 0.0,
            l_scale,
            a_scale,
            twist_pub,
        }
    }

    pub fn key_loop(&mut self) {
        println!("Reading from keyboard");
        println!("---------------------------");
        println!("Use arrow keys to move the turtle.");
        println!("'q' to quit.");

        let _stdout = io::stdout().into_raw_mode().unwrap();
        let stdin = io::stdin();
        for key in stdin.keys() {
            self.linear = 0.0;
            self.angular = 0.0;

            match key.unwrap() {
                Key::Left => {
                    self.angular = 1.0;
                }
                Key::Right => {
                    self.angular = -1.0;
                }
                Key::Up => {
                    self.linear = 1.0;
                }
                Key::Down => {
                    self.linear = -1.0;
                }
                Key::Char('q') | Key::Ctrl('c') => {
                    break;
                }
                _ => {}
            }
            let mut twist_msg = geometry_msgs::msg::Twist::default();
            twist_msg.angular.z = self.angular * self.a_scale;
            twist_msg.linear.x = self.linear * self.l_scale;
            self.twist_pub.publish(twist_msg).unwrap();
        }
    }
}

fn main() -> Result<(), rclrs::RclrsError> {
    let context = rclrs::Context::new(env::args()).unwrap();

    let mut teleop_turtle = TeleopTurtle::new(&context);

    teleop_turtle.key_loop();

    Ok(())
}

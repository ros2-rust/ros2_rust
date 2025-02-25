use rclrs::*;
use std::{sync::Arc, thread, time::Duration};
use std_msgs::msg::String as StringMsg;

struct SimplePublisher {
    publisher: Arc<Publisher<StringMsg>>,
}

impl SimplePublisher {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("simple_publisher").unwrap();
        let publisher = node
            .create_publisher("publish_hello", QOS_PROFILE_DEFAULT)
            .unwrap();
        Ok(Self { publisher })
    }

    fn publish_data(&self, increment: i32) -> Result<i32, RclrsError> {
        let msg: StringMsg = StringMsg {
            data: format!("Hello World {}", increment),
        };
        self.publisher.publish(msg).unwrap();
        Ok(increment + 1_i32)
    }
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env().unwrap().create_basic_executor();
    let publisher = Arc::new(SimplePublisher::new(&executor).unwrap());
    let publisher_other_thread = Arc::clone(&publisher);
    let mut count: i32 = 0;
    thread::spawn(move || loop {
        thread::sleep(Duration::from_millis(1000));
        count = publisher_other_thread.publish_data(count).unwrap();
    });
    executor.spin(SpinOptions::default()).first_error()
}

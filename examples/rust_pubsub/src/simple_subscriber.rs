use rclrs::*;
use std::{
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};
use std_msgs::msg::String as StringMsg;

pub struct SimpleSubscriptionNode {
    _subscriber: Arc<Subscription<StringMsg>>,
    data: Arc<Mutex<Option<StringMsg>>>,
}

impl SimpleSubscriptionNode {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("simple_subscription").unwrap();
        let data: Arc<Mutex<Option<StringMsg>>> = Arc::new(Mutex::new(None));
        let data_mut: Arc<Mutex<Option<StringMsg>>> = Arc::clone(&data);
        let _subscriber = node
            .create_subscription::<StringMsg, _>(
                "publish_hello",
                QOS_PROFILE_DEFAULT,
                move |msg: StringMsg| {
                    *data_mut.lock().unwrap() = Some(msg);
                },
            )
            .unwrap();
        Ok(Self { _subscriber, data })
    }
    fn data_callback(&self) -> Result<(), RclrsError> {
        if let Some(data) = self.data.lock().unwrap().as_ref() {
            println!("{}", data.data);
        } else {
            println!("No message available yet.");
        }
        Ok(())
    }
}
fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env().unwrap().create_basic_executor();
    let subscription = Arc::new(SimpleSubscriptionNode::new(&executor).unwrap());
    let subscription_other_thread = Arc::clone(&subscription);
    thread::spawn(move || loop {
        thread::sleep(Duration::from_millis(1000));
        subscription_other_thread.data_callback().unwrap()
    });
    executor.spin(SpinOptions::default()).first_error()
}

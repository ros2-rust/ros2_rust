use rclrs::*;
use std::{
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};
use std_msgs::msg::String as StringMsg;

pub struct SimpleSubscriptionNode {
    #[allow(unused)]
    subscriber: Subscription<StringMsg>,
    data: Arc<Mutex<Option<StringMsg>>>,
}

impl SimpleSubscriptionNode {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("simple_subscription").unwrap();
        let data: Arc<Mutex<Option<StringMsg>>> = Arc::new(Mutex::new(None));
        let data_mut: Arc<Mutex<Option<StringMsg>>> = Arc::clone(&data);
        let subscriber = node
            .create_subscription::<StringMsg, _>(
                "publish_hello",
                move |msg: StringMsg| {
                    *data_mut.lock().unwrap() = Some(msg);
                },
            )
            .unwrap();
        Ok(Self { subscriber, data })
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
    let node = SimpleSubscriptionNode::new(&executor).unwrap();

    // TODO(@mxgrey): Replace this thread with a timer when the Timer feature
    // gets merged.
    thread::spawn(move || loop {
        thread::sleep(Duration::from_millis(1000));
        node.data_callback().unwrap()
    });
    executor.spin(SpinOptions::default()).first_error()
}

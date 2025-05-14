use rclrs::*;
use std::{thread, time::Duration};
use std_msgs::msg::String as StringMsg;

pub struct SimpleSubscriptionNode {
    #[allow(unused)]
    subscriber: WorkerSubscription<StringMsg, Option<String>>,
    worker: Worker<Option<String>>,
}

impl SimpleSubscriptionNode {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("simple_subscription").unwrap();
        let worker = node.create_worker(None);

        let subscriber = worker
            .create_subscription::<StringMsg, _>(
                "publish_hello",
                move |data: &mut Option<String>, msg: StringMsg| {
                    *data = Some(msg.data);
                },
            )
            .unwrap();

        Ok(Self { subscriber, worker })
    }
}
fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env().unwrap().create_basic_executor();
    let node = SimpleSubscriptionNode::new(&executor).unwrap();

    // TODO(@mxgrey): Replace this thread with a timer when the Timer feature
    // gets merged.
    thread::spawn(move || loop {
        thread::sleep(Duration::from_secs(1));
        let _ = node.worker.run(|data: &mut Option<String>| {
            if let Some(data) = data {
                println!("{data}");
            } else {
                println!("No message available yet.");
            }
        });
    });

    executor.spin(SpinOptions::default()).first_error()
}

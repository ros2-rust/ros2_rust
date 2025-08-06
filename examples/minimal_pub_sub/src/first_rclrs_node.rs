use rclrs::*;
use std::sync::{Arc, Mutex};
use std_msgs::msg::String as StringMsg;

struct RepublisherNode {
    _node: Arc<rclrs::Node>,
    _subscription: Arc<rclrs::Subscription<StringMsg>>,
    _publisher: Arc<rclrs::Publisher<StringMsg>>,
    _data: Arc<Mutex<Option<StringMsg>>>,
}

impl RepublisherNode {
    fn new(executor: &rclrs::Executor) -> Result<Self, rclrs::RclrsError> {
        let _node = executor.create_node("republisher")?;
        let _data = Arc::new(Mutex::new(None));
        let data_cb = Arc::clone(&_data);
        let _subscription = _node.create_subscription(
            "in_topic".keep_last(10).transient_local(),
            move |msg: StringMsg| {
                *data_cb.lock().unwrap() = Some(msg);
            },
        )?;
        let _publisher = _node.create_publisher::<std_msgs::msg::String>("out_topic")?;
        Ok(Self {
            _node,
            _subscription,
            _publisher,
            _data,
        })
    }

    fn republish(&self) -> Result<(), rclrs::RclrsError> {
        if let Some(s) = &*self._data.lock().unwrap() {
            self._publisher.publish(s)?;
        }
        Ok(())
    }
}

fn main() -> Result<(), rclrs::RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let _republisher = RepublisherNode::new(&executor)?;
    std::thread::spawn(move || -> Result<(), rclrs::RclrsError> {
        loop {
            use std::time::Duration;
            std::thread::sleep(Duration::from_millis(1000));
            _republisher.republish()?;
        }
    });
    executor
        .spin(SpinOptions::default())
        .first_error()
        .map_err(|err| err.into())
}

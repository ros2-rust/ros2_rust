use std::{
    env,
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

use anyhow::Result;

struct Publisher {
    publisher: Arc<Mutex<rclrs::Publisher<std_msgs::msg::String>>>,
    publish_count: Arc<Mutex<u32>>,
}

unsafe impl Send for Publisher {}

impl Publisher {
    pub fn new(context: &rclrs::Context) -> Self {
        let node = rclrs::create_node(context, "publisher").unwrap();

        let publisher = node
            .create_publisher::<std_msgs::msg::String>("topic", rclrs::QOS_PROFILE_DEFAULT)
            .unwrap();

        Self {
            publisher: Arc::new(Mutex::new(publisher)),
            publish_count: Arc::new(Mutex::new(0)),
        }
    }

    fn init(&mut self) {
        let publish_count = self.publish_count.clone();
        let publisher = self.publisher.clone();

        thread::spawn(move || loop {
            thread::sleep(Duration::from_secs(1));

            let msg = std_msgs::msg::String {
                data: format!("Hello, world! {}", publish_count.lock().unwrap()),
            };

            println!("Publishing: [{}]", msg.data);

            publisher.lock().unwrap().publish(msg).unwrap();
            let mut num_count = publish_count.lock().unwrap();
            *num_count += 1;
        });
    }
}

fn main() -> Result<(), rclrs::RclrsError> {
    let context = rclrs::Context::new(env::args())?;
    let mut publisher = Publisher::new(&context);
    publisher.init();
    while context.ok() {
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
    Ok(())
}

/// Creates a SimpleTimerNode, initializes a node and the timer with a callback
/// that prints the timer callback execution iteration. The callback is executed
/// thanks to the spin, which is in charge of executing the timer's events among
/// other entities' events.
use rclrs::{create_node, Context, Node, RclrsError, Timer};
use std::{
    env,
    sync::{Arc, Mutex},
};

/// Contains both the node and timer.
struct SimpleTimerNode {
    node: Arc<Node>,
    timer: Arc<Timer>,
}

impl SimpleTimerNode {
    /// Creates a node and a timer with a callback.
    ///
    /// The callback will simply print to stdout:
    /// "Drinking 🧉 for the xth time every p nanoseconds."
    /// where x is the iteration callback counter and p is the period of the timer.
    fn new(context: &Context, timer_period_ns: i64) -> Result<Self, RclrsError> {
        let node = create_node(context, "simple_timer_node")?;
        let count: Arc<Mutex<i32>> = Arc::new(Mutex::new(0));
        let timer = node.create_timer(
            timer_period_ns,
            context,
            Some(Box::new(move |_| {
                let x = *count.lock().unwrap();
                println!(
                    "Drinking 🧉 for the {}th time every {} nanoseconds.",
                    x, timer_period_ns
                );
                *count.lock().unwrap() = x + 1;
            })),
            None,
        )?;
        Ok(Self { node, timer })
    }
}

fn main() -> Result<(), RclrsError> {
    let timer_period: i64 = 1e9 as i64; // 1 seconds.
    let context = Context::new(env::args()).unwrap();
    let simple_timer_node = Arc::new(SimpleTimerNode::new(&context, timer_period).unwrap());
    rclrs::spin(simple_timer_node.node.clone())
}

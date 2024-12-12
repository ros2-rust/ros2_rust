/// Creates a SimpleTimerNode, initializes a node and the timer with a callback
/// that prints the timer callback execution iteration. The callback is executed
/// thanks to the spin, which is in charge of executing the timer's events among
/// other entities' events.
use rclrs::{create_node, Context, Node, RclrsError, Timer};
use std::{
    env,
    sync::Arc,
    time::Duration,
};

/// Contains both the node and timer.
struct SimpleTimerNode {
    node: Arc<Node>,
    #[allow(unused)]
    timer: Arc<Timer>,
}

impl SimpleTimerNode {
    /// Creates a node and a timer with a callback.
    ///
    /// The callback will simply print to stdout:
    /// "Drinking 🧉 for the xth time every p nanoseconds."
    /// where x is the iteration callback counter and p is the period of the timer.
    fn new(context: &Context, timer_period: Duration) -> Result<Self, RclrsError> {
        let node = create_node(context, "simple_timer_node")?;
        let mut x = 0;
        let timer = node.create_timer_repeating(
            timer_period,
            move || {
                x += 1;
                println!(
                    "Drinking 🧉 for the {x}th time every {:?}.",
                    timer_period,
                );
            },
        )?;
        Ok(Self { node, timer })
    }
}

fn main() -> Result<(), RclrsError> {
    let context = Context::new(env::args()).unwrap();
    let simple_timer_node = Arc::new(SimpleTimerNode::new(&context, Duration::from_secs(1)).unwrap());
    rclrs::spin(simple_timer_node.node.clone())
}

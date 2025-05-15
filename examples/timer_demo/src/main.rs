/// Creates a SimpleTimerNode, initializes a node and the timer with a callback
/// that prints the timer callback execution iteration. The callback is executed
/// thanks to the spin, which is in charge of executing the timer's events among
/// other entities' events.
use rclrs::*;
use std::time::Duration;

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("timer_demo")?;
    let worker = node.create_worker::<usize>(0);
    let timer_period = Duration::from_secs(1);
    let _timer = worker.create_timer_repeating(timer_period, move |count: &mut usize| {
        *count += 1;
        println!(
            "Drinking 🧉 for the {}th time every {:?}.",
            *count, timer_period,
        );
    })?;

    executor.spin(SpinOptions::default()).first_error()
}

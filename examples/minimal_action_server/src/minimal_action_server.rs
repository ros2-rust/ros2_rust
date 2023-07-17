use std::env;
use std::sync::Arc;
use std::thread;

use anyhow::{Error, Result};

type Fibonacci = example_interfaces::action::Fibonacci;
type GoalHandleFibonacci = rclrs::ServerGoalHandle<Fibonacci>;

fn handle_goal(
    _uuid: &rclrs::GoalUUID,
    goal: Arc<example_interfaces::action::rmw::Fibonacci_Goal>,
) -> rclrs::GoalResponse {
    println!("Received goal request with order {}", goal.order);
    if goal.order > 9000 {
        rclrs::GoalResponse::Reject
    } else {
        rclrs::GoalResponse::AcceptAndExecute
    }
}

fn handle_cancel(_goal_handle: Arc<GoalHandleFibonacci>) -> rclrs::CancelResponse {
    println!("Got request to cancel goal");
    rclrs::CancelResponse::Accept
}

fn execute(goal_handle: Arc<GoalHandleFibonacci>) {
    println!("Executing goal");
    thread::sleep(std::time::Duration::from_millis(100));
}

fn handle_accepted(goal_handle: Arc<GoalHandleFibonacci>) {
    thread::spawn(move || {
        execute(goal_handle);
    });
}

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;

    let mut node = rclrs::create_node(&context, "minimal_action_server")?;

    let _action_server = node.create_action_server::<example_interfaces::action::Fibonacci>(
        "fibonacci",
        handle_goal,
        handle_cancel,
        handle_accepted,
    );

    rclrs::spin(&node).map_err(|err| err.into())
}

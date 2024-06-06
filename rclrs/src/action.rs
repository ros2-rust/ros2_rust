mod client;
mod server;
mod server_goal_handle;

use crate::rcl_bindings::RCL_ACTION_UUID_SIZE;

pub use client::{ActionClient, ActionClientBase};
pub use server::{ActionServer, ActionServerBase};
pub use server_goal_handle::ServerGoalHandle;

pub type GoalUUID = [u8; RCL_ACTION_UUID_SIZE];

pub enum GoalResponse {
    Reject = 1,
    AcceptAndExecute = 2,
    AcceptAndDefer = 3,
}

pub enum CancelResponse {
    Reject = 1,
    Accept = 2,
}

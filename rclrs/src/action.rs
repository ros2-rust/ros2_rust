pub(crate) mod client;
pub(crate) mod server;
mod server_goal_handle;

use crate::rcl_bindings::RCL_ACTION_UUID_SIZE;
use std::fmt;

pub use client::{ActionClient, ActionClientBase};
pub use server::{ActionServer, ActionServerBase};
pub use server_goal_handle::ServerGoalHandle;

/// A unique identifier for a goal request.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct GoalUuid(pub [u8; RCL_ACTION_UUID_SIZE]);

impl fmt::Display for GoalUuid {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        write!(f, "{:02x}{:02x}{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
               self.0[0],
               self.0[1],
               self.0[2],
               self.0[3],
               self.0[4],
               self.0[5],
               self.0[6],
               self.0[7],
               self.0[8],
               self.0[9],
               self.0[10],
               self.0[11],
               self.0[12],
               self.0[13],
               self.0[14],
               self.0[15],
               )
    }
}

/// The response returned by an [`ActionServer`]'s goal callback when a goal request is received.
pub enum GoalResponse {
    /// The goal is rejected and will not be executed.
    Reject = 1,
    /// The server accepts the goal and will begin executing it immediately.
    AcceptAndExecute = 2,
    /// The server accepts the goal and will begin executing it later.
    AcceptAndDefer = 3,
}

/// The response returned by an [`ActionServer`]'s cancel callback when a goal is requested to be cancelled.
pub enum CancelResponse {
    /// The server will not try to cancel the goal.
    Reject = 1,
    /// The server will try to cancel the goal.
    Accept = 2,
}

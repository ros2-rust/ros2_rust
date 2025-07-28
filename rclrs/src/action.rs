use std::ops::Deref;

pub(crate) mod action_client;
pub(crate) mod action_server;
mod action_server_goal_handle;

use crate::rcl_bindings::RCL_ACTION_UUID_SIZE;
use std::fmt;

pub use action_client::*;
pub use action_server::*;
use action_server_goal_handle::*;

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

impl Deref for GoalUuid {
    type Target = [u8; RCL_ACTION_UUID_SIZE];

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

/// The response returned by an [`ActionServer`]'s cancel callback when a goal is requested to be cancelled.
#[derive(PartialEq, Eq)]
pub enum CancelResponse {
    /// The server will not try to cancel the goal.
    Reject = 1,
    /// The server will try to cancel the goal.
    Accept = 2,
}

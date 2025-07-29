use std::ops::Deref;

pub(crate) mod action_client;
pub use action_client::*;

pub(crate) mod action_goal_receiver;
pub use action_goal_receiver::*;

pub(crate) mod action_server;
pub use action_server::*;

use crate::rcl_bindings::RCL_ACTION_UUID_SIZE;
use std::fmt;


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

/// Values defined by `action_msgs/msg/GoalStatus`
#[repr(i8)]
#[derive(Debug, Clone, Hash, PartialEq, Eq)]
pub enum GoalStatus {
    /// The goal status has never been initialized. This likely means it has not
    /// yet been accepted.
    Unknown = 0,
    /// The goal was accepted by the action server.
    Accepted = 1,
    /// The goal is being executed by the action server.
    Executing = 2,
    /// The action server has accepting cancelling the goal and is in the process
    /// of cancelling it.
    Cancelling = 3,
    /// The action server has successfully reached the goal.
    Succeeded = 4,
    /// The action server has finished cancelling the goal.
    Cancelled = 5,
    /// The action server has aborted the goal. This suggests an error happened
    /// during execution or cancelling.
    Aborted = 6,
}

use std::ops::Deref;

pub(crate) mod action_client;
pub use action_client::*;

pub(crate) mod action_goal_receiver;
pub use action_goal_receiver::*;

pub(crate) mod action_server;
pub use action_server::*;

use crate::{
    rcl_bindings::RCL_ACTION_UUID_SIZE,
    vendor::builtin_interfaces::msg::Time,
    log_error,
};
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
#[derive(Debug, Clone, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub enum GoalStatusCode {
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

impl From<i8> for GoalStatusCode {
    fn from(value: i8) -> Self {
        if value <= 0 && value <= 6 {
            unsafe {
                // SAFETY: We have already ensured that the integer value is
                // within the acceptable range for the enum, so transmuting is
                // safe.
                return std::mem::transmute(value);
            }
        }

        log_error!(
            "goal_status_code.from",
            "Invalid integer value being cast to a goal status code: {value}. \
            Values should be in the range [0, 6]. We will set this as 0 (Unknown).",
        );
        GoalStatusCode::Unknown
    }
}

/// A status update for a goal. Includes the status code, the goal uuid, and the
/// timestamp of when the status was set by the action server.
#[derive(Debug, Clone, PartialEq, PartialOrd)]
pub struct GoalStatus {
    /// The status code describing what status was set by the action server.
    pub code: GoalStatusCode,
    /// The uuid of the goal whose status was updated.
    pub goal_id: GoalUuid,
    /// Time that the status was set by the action server. The time measured by
    /// the action server might not align with the time measured by the action
    /// client, so care should be taken when using this time value.
    pub stamp: Time,
}

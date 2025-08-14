use crate::GoalUuid;

/// This represents a goal that has reached a terminated state. This struct is
/// used as the return value for action server callbacks to guide the user to
/// bring the goal to a terminal state.
///
/// It is safe to allow this struct to drop if you are using an action goal
/// receiver instead of an action server callback.
pub struct TerminatedGoal {
    pub(super) uuid: GoalUuid,
}

impl TerminatedGoal {
    /// Get the UUID of the goal that was terminated.
    pub fn uuid(&self) -> &GoalUuid {
        &self.uuid
    }
}

use std::{collections::HashMap, ops::Deref};

pub(crate) mod action_client;
pub use action_client::*;

pub(crate) mod action_goal_receiver;
pub use action_goal_receiver::*;

pub(crate) mod action_server;
pub use action_server::*;

use crate::{
    rcl_bindings::*,
    vendor::builtin_interfaces::msg::Time,
    DropGuard,
    log_error,
};
use std::fmt;


/// A unique identifier for a goal request.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct GoalUuid(pub [u8; RCL_ACTION_UUID_SIZE]);

impl GoalUuid {
    /// A zeroed-out goal ID has a special meaning for cancellation requests
    /// which indicates that no specific goal is being requested.
    fn zero() -> Self {
        Self([0; RCL_ACTION_UUID_SIZE])
    }
}

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

impl From<[u8; RCL_ACTION_UUID_SIZE]> for GoalUuid {
    fn from(value: [u8; RCL_ACTION_UUID_SIZE]) -> Self {
        Self(value)
    }
}

impl From<&[u8; RCL_ACTION_UUID_SIZE]> for GoalUuid {
    fn from(value: &[u8; RCL_ACTION_UUID_SIZE]) -> Self {
        Self(*value)
    }
}

/// The response returned by an [`ActionServer`]'s cancel callback when a goal is requested to be cancelled.
#[repr(i8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum CancelResponseCode {
    /// The server will try to cancel the goal.
    Accept = 0,
    /// The server will not try to cancel the goal.
    Reject = 1,
    /// The requested goal is unknown.
    UnknownGoal = 2,
    /// The goal already reached a terminal state.
    GoalTerminated = 3,
}

impl CancelResponseCode {
    /// Check if the cancellation was accepted.
    pub fn is_accepted(&self) -> bool {
        matches!(self, Self::Accept)
    }
}

impl From<i8> for CancelResponseCode {
    fn from(value: i8) -> Self {
        if 0 <= value && value <= 3 {
            unsafe {
                // SAFETY: We have already ensured that the integer value is
                // within the acceptable range for the enum, so transmuting is
                // safe.
                return std::mem::transmute(value);
            }
        }

        log_error!(
            "cancel_response.from",
            "Invalid integer value being cast to a cancel response: {value}. \
            Values should be in the range [0, 3]. We will set this as 1 (Reject).",
        );
        CancelResponseCode::Reject
    }
}

/// This is returned by [`CancellationClient`] to inform whether a cancellation
/// of a single goal was successful.
///
/// When a cancellation request might cancel multiple goals, [`MultiCancelResponse`]
/// will be used.
#[derive(Debug, Clone, PartialEq, PartialOrd)]
pub struct CancelResponse {
    /// What kind of response was given.
    pub code: CancelResponseCode,
    /// What time the response took effect according to the action server.
    /// This will be default-initialized if no goal was cancelled.
    pub stamp: Option<Time>,
}

impl CancelResponse {
    /// Check whether the request was accepted.
    pub fn is_accepted(&self) -> bool {
        self.code.is_accepted()
    }
}

/// This is returned by [`ActionClientState::cancel_all_goals`] and
/// [`ActionClientState::cancel_goals_prior_to`].
#[derive(Debug, Clone, PartialEq)]
pub struct MultiCancelResponse {
    /// What kind of response was given.
    pub code: CancelResponseCode,
    /// The time stamp that the response took effect for each goal that is being
    /// cancelled. If the request was not accepted then this may be empty.
    pub stamps: HashMap<GoalUuid, Time>,
}

impl MultiCancelResponse {
    /// Check whether the request was accepted.
    pub fn is_accepted(&self) -> bool {
        self.code.is_accepted()
    }
}

/// Values defined by `action_msgs/msg/GoalStatus`
#[repr(i8)]
#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord)]
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
        if 0 <= value && value <= 6 {
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

fn empty_goal_status_array() -> DropGuard<rcl_action_goal_status_array_t> {
    DropGuard::new(
        unsafe {
            // SAFETY: No preconditions
            let mut array = rcl_action_get_zero_initialized_goal_status_array();
            array.allocator = rcutils_get_default_allocator();
            array
        },
        |mut goal_statuses| unsafe {
            // SAFETY: The goal_status array is either zero-initialized and empty or populated by
            // `rcl_action_get_goal_status_array`. In either case, it can be safely finalized.
            rcl_action_goal_status_array_fini(&mut goal_statuses);
        }
    )
}

#[cfg(test)]
mod tests {
    use crate::*;
    use example_interfaces::action::{Fibonacci, Fibonacci_Goal, Fibonacci_Result, Fibonacci_Feedback};
    use tokio::sync::mpsc::unbounded_channel;
    use futures::StreamExt;
    use std::time::Duration;

    #[test]
    fn test_action_success() {
        let mut executor = Context::default().create_basic_executor();

        let node = executor.create_node(&format!("test_action_success_{}", line!())).unwrap();
        let action_name = format!("test_action_success_{}_action", line!());
        let _action_server = node.create_action_server(
            &action_name,
            fibonacci_action,
        ).unwrap();

        let client = node.create_action_client::<Fibonacci>(&action_name).unwrap();

        let order_10_sequence = [1, 1, 2, 3, 5, 8, 13, 21, 34, 55];

        let request = client.request_goal(Fibonacci_Goal {
            order: 10,
        });

        let promise = executor.commands().run(async move {
            let mut goal_client_stream = request.await.unwrap().stream();
            let mut expected_feedback_len = 0;
            while let Some(event) = goal_client_stream.next().await {
                match event {
                    GoalEvent::Feedback(feedback) => {
                        expected_feedback_len += 1;
                        assert_eq!(feedback.sequence.len(), expected_feedback_len);
                    }
                    GoalEvent::Status(s) => {
                        assert!(
                            matches!(s.code, GoalStatusCode::Unknown | GoalStatusCode::Executing | GoalStatusCode::Succeeded),
                            "Actual code: {:?}",
                            s.code,
                        );
                    }
                    GoalEvent::Result((status, result)) => {
                        assert_eq!(status, GoalStatusCode::Succeeded);
                        assert_eq!(result.sequence, order_10_sequence);
                        return;
                    }
                }
            }
        });

        executor.spin(SpinOptions::default().until_promise_resolved(promise));

        let request = client.request_goal(Fibonacci_Goal {
            order: 10,
        });

        let promise = executor.commands().run(async move {
            let (status, result) = request.await.unwrap().result.await;
            assert_eq!(status, GoalStatusCode::Succeeded);
            assert_eq!(result.sequence, order_10_sequence);
        });

        executor.spin(SpinOptions::default().until_promise_resolved(promise));
    }

    async fn fibonacci_action(handle: RequestedGoal<Fibonacci>) -> TerminatedGoal {
        let goal_order = handle.goal().order;
        if goal_order < 0 {
            return handle.reject();
        }

        let mut result = Fibonacci_Result::default();

        let executing = match handle.accept().begin() {
            BeginAcceptedGoal::Execute(executing) => executing,
            BeginAcceptedGoal::Cancel(cancelling) => {
                return cancelling.cancelled_with(result);
            }
        };

        let (sender, mut receiver) = unbounded_channel();
        std::thread::spawn(move || {

            let mut previous = 0;
            let mut current = 1;

            for _ in 0..goal_order {
                if let Err(_) = sender.send(current) {
                    // The action has been cancelled early, so just drop this thread.
                    return;
                }

                let next = previous + current;
                previous = current;
                current = next;
                std::thread::sleep(Duration::from_micros(10));
            }
        });

        let mut sequence = Vec::new();
        loop {
            match executing.unless_cancel_requested(receiver.recv()).await {
                Ok(Some(next)) => {
                    // We have a new item in the sequence
                    sequence.push(next);
                    executing.publish_feedback(
                        Fibonacci_Feedback {
                            sequence: sequence.clone(),
                        }
                    );
                }
                Ok(None) => {
                    // The sequence has finished
                    result.sequence = sequence;
                    return executing.succeeded_with(result);
                }
                Err(_) => {
                    // The action has been cancelled
                    result.sequence = sequence;
                    return executing.begin_cancelling().cancelled_with(result);
                }
            }
        }
    }
}

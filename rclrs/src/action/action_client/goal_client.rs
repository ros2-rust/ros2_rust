use crate::{
    vendor::builtin_interfaces::msg::Time,
    CancellationClient, FeedbackClient, StatusClient, ResultClient,
};
use rosidl_runtime_rs::Action;

/// The goal client bundles a set of receivers that will allow you to await
/// different information from the action server, such as feedback messages,
/// status updates, and the final result. It also provides a way to request
/// cancelling a goal.
///
/// This struct is designed to be [destructured][1] so each of its fields can be
/// used independently. This is important because many of them require mutable
/// access (or one-time access) to be used, so being blocked behind `&mut self`
/// would make it impossible to use them independently, e.g. to await more than
/// one at a time.
///
/// [1]: https://doc.rust-lang.org/rust-by-example/flow_control/match/destructuring/destructure_structures.html
pub struct GoalClient<A: Action> {
    /// Receive feedback messages for the goal.
    pub feedback: FeedbackClient<A>,
    /// Watch the status of the goal.
    pub status: StatusClient<A>,
    /// Get the final result of the goal.
    pub result: ResultClient<A>,
    /// Use this if you want to request the goal to be cancelled.
    pub cancellation: CancellationClient<A>,
    /// The time that the goal was accepted.
    pub stamp: Time,
}

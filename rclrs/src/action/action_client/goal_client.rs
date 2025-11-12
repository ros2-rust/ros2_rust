use crate::{
    vendor::builtin_interfaces::msg::Time, CancellationClient, FeedbackClient, GoalStatus,
    GoalStatusCode, ResultClient, StatusWatcher,
};
use rosidl_runtime_rs::Action;
use std::{
    pin::Pin,
    task::{Context, Poll},
};
use tokio_stream::{Stream, StreamMap};

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
    pub status: StatusWatcher<A>,
    /// Get the final result of the goal.
    pub result: ResultClient<A>,
    /// Use this if you want to request the goal to be cancelled.
    pub cancellation: CancellationClient<A>,
    /// The time that the goal was accepted.
    pub stamp: Time,
}

impl<A: Action> Clone for GoalClient<A> {
    fn clone(&self) -> Self {
        Self {
            feedback: self.feedback.clone(),
            status: self.status.clone(),
            result: self.result.clone(),
            cancellation: self.cancellation.clone(),
            stamp: self.stamp.clone(),
        }
    }
}

impl<A: Action> GoalClient<A> {
    /// Create a concurrent stream that will emit events related to this goal.
    pub fn stream(self) -> GoalClientStream<A> {
        let Self {
            mut feedback,
            mut status,
            result,
            ..
        } = self;

        let rx_feedback = Box::pin(async_stream::stream! {
            while let Some(msg) = feedback.recv().await {
                yield GoalEvent::Feedback(msg);
            }
        }) as Pin<Box<dyn Stream<Item = GoalEvent<A>> + Send>>;

        let rx_status = Box::pin(async_stream::stream! {
            let initial_value = (*status.borrow_and_update()).clone();
            yield GoalEvent::Status(initial_value);

            while let Ok(_) = status.changed().await {
                let value = (*status.borrow_and_update()).clone();
                yield GoalEvent::Status(value);
            }
        }) as Pin<Box<dyn Stream<Item = GoalEvent<A>> + Send>>;

        let rx_result = Box::pin(async_stream::stream! {
            yield GoalEvent::Result(result.await);
        }) as Pin<Box<dyn Stream<Item = GoalEvent<A>> + Send>>;

        let mut stream_map: StreamMap<i32, Pin<Box<dyn Stream<Item = GoalEvent<A>> + Send>>> =
            StreamMap::new();
        stream_map.insert(0, rx_feedback);
        stream_map.insert(1, rx_status);
        stream_map.insert(2, rx_result);
        GoalClientStream { stream_map }
    }
}

/// Use this to receive a stream of messages from the action server which may be
/// a mix of feedback messages, status updates, and the final result. These are
/// multiplexed by the [`GoalEvent`] enum.
///
/// Remember to `use futures::StreamExt;` in order to use the streaming trait
/// of this struct.
///
/// # Example
///
/// ```
/// use rclrs::*;
/// use crate::rclrs::vendor::example_interfaces::action::Fibonacci;
/// use futures::StreamExt;
///
/// async fn process_goal_client_stream(
///     mut goal_client_stream: GoalClientStream<Fibonacci>
/// ) {
///     while let Some(event) = goal_client_stream.next().await {
///         match event {
///             GoalEvent::Feedback(feedback) => {
///                 println!("Received feedback: {feedback:?}");
///             }
///             GoalEvent::Status(status) => {
///                 println!("Received status update: {status:?}");
///             }
///             GoalEvent::Result((status, result)) => {
///                 println!("Received the final status: {status:?}");
///                 println!("The result was: {result:?}");
///                 break;
///             }
///         }
///     }
/// }
/// ```
pub struct GoalClientStream<A: Action> {
    stream_map: StreamMap<i32, Pin<Box<dyn Stream<Item = GoalEvent<A>> + Send>>>,
}

impl<A: Action> Stream for GoalClientStream<A> {
    type Item = GoalEvent<A>;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context) -> Poll<Option<Self::Item>> {
        Stream::poll_next(Pin::new(&mut self.get_mut().stream_map), cx)
            .map(|r| r.map(|(_, event)| event))
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        self.stream_map.size_hint()
    }
}

/// Any of the possible update events that can happen for a goal.
pub enum GoalEvent<A: Action> {
    /// A feedback message was received
    Feedback(A::Feedback),
    /// A status update was received
    Status(GoalStatus),
    /// The result of the goal was received
    Result((GoalStatusCode, A::Result)),
}

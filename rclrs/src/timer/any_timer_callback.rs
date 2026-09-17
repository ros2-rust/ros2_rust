use crate::{TimerState, WorkScope};
use std::sync::Arc;

type RepeatingCallback<Scope> =
    Box<dyn FnMut(&mut <Scope as WorkScope>::Payload, &Arc<TimerState<Scope>>) + Send>;
type OneShotCallback<Scope> =
    Box<dyn FnOnce(&mut <Scope as WorkScope>::Payload, &Arc<TimerState<Scope>>) + Send>;

/// A callback that can be triggered when a timer elapses.
pub enum AnyTimerCallback<Scope: WorkScope> {
    /// This callback will be triggered repeatedly, each time the period of the
    /// timer elapses.
    Repeating(RepeatingCallback<Scope>),
    /// This callback will be triggered exactly once, the first time the period
    /// of the timer elapses.
    OneShot(OneShotCallback<Scope>),
    /// Do nothing when the timer elapses. This can be replaced later so that
    /// the timer does something.
    Inert,
}

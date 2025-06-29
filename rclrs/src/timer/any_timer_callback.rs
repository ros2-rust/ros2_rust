use crate::{TimerState, WorkScope};
use std::sync::Arc;

/// A callback that can be triggered when a timer elapses.
pub enum AnyTimerCallback<Scope: WorkScope> {
    /// This callback will be triggered repeatedly, each time the period of the
    /// timer elapses.
    Repeating(Box<dyn FnMut(&mut Scope::Payload, &Arc<TimerState<Scope>>) + Send>),
    /// This callback will be triggered exactly once, the first time the period
    /// of the timer elapses.
    OneShot(Box<dyn FnOnce(&mut Scope::Payload, &Arc<TimerState<Scope>>) + Send>),
    /// Do nothing when the timer elapses. This can be replaced later so that
    /// the timer does something.
    Inert,
}

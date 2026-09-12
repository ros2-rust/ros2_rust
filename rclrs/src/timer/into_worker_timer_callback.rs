use crate::{AnyTimerCallback, Time, TimerState, WorkScope};
use std::sync::Arc;

/// This trait is used to create timer callbacks for repeating timers in a Worker.
pub trait IntoWorkerTimerRepeatingCallback<Scope: WorkScope, Args>: 'static + Send {
    /// Convert a suitable object into a repeating timer callback for a worker scope
    fn into_worker_timer_repeating_callback(self) -> AnyTimerCallback<Scope>;
}

impl<Scope: WorkScope, Func> IntoWorkerTimerRepeatingCallback<Scope, ()> for Func
where
    Func: FnMut() + 'static + Send,
{
    fn into_worker_timer_repeating_callback(mut self) -> AnyTimerCallback<Scope> {
        AnyTimerCallback::Repeating(Box::new(move |_, _| self()))
    }
}

impl<Scope: WorkScope, Func> IntoWorkerTimerRepeatingCallback<Scope, (Scope::Payload,)> for Func
where
    Func: FnMut(&mut Scope::Payload) + 'static + Send,
{
    fn into_worker_timer_repeating_callback(mut self) -> AnyTimerCallback<Scope> {
        AnyTimerCallback::Repeating(Box::new(move |payload, _| self(payload)))
    }
}

impl<Scope: WorkScope, Func>
    IntoWorkerTimerRepeatingCallback<Scope, (Scope::Payload, Arc<TimerState<Scope>>)> for Func
where
    Func: FnMut(&mut Scope::Payload, &Arc<TimerState<Scope>>) + 'static + Send,
{
    fn into_worker_timer_repeating_callback(self) -> AnyTimerCallback<Scope> {
        AnyTimerCallback::Repeating(Box::new(self))
    }
}

impl<Scope: WorkScope, Func> IntoWorkerTimerRepeatingCallback<Scope, (Scope::Payload, Time)>
    for Func
where
    Func: FnMut(&mut Scope::Payload, Time) + 'static + Send,
{
    fn into_worker_timer_repeating_callback(mut self) -> AnyTimerCallback<Scope> {
        AnyTimerCallback::Repeating(Box::new(move |payload, t| {
            self(payload, t.handle.clock.now())
        }))
    }
}

/// This trait is used to create timer callbacks for one-shot timers in a Worker.
pub trait IntoWorkerTimerOneshotCallback<Scope: WorkScope, Args>: 'static + Send {
    /// Convert a suitable object into a one-shot timer callback for a worker scope
    fn into_worker_timer_oneshot_callback(self) -> AnyTimerCallback<Scope>;
}

impl<Scope: WorkScope, Func> IntoWorkerTimerOneshotCallback<Scope, ()> for Func
where
    Func: FnOnce() + 'static + Send,
{
    fn into_worker_timer_oneshot_callback(self) -> AnyTimerCallback<Scope> {
        AnyTimerCallback::OneShot(Box::new(move |_, _| self()))
    }
}

impl<Scope: WorkScope, Func> IntoWorkerTimerOneshotCallback<Scope, (Scope::Payload,)> for Func
where
    Func: FnOnce(&mut Scope::Payload) + 'static + Send,
{
    fn into_worker_timer_oneshot_callback(self) -> AnyTimerCallback<Scope> {
        AnyTimerCallback::OneShot(Box::new(move |payload, _| self(payload)))
    }
}

impl<Scope: WorkScope, Func>
    IntoWorkerTimerOneshotCallback<Scope, (Scope::Payload, Arc<TimerState<Scope>>)> for Func
where
    Func: FnOnce(&mut Scope::Payload, &Arc<TimerState<Scope>>) + 'static + Send,
{
    fn into_worker_timer_oneshot_callback(self) -> AnyTimerCallback<Scope> {
        AnyTimerCallback::OneShot(Box::new(self))
    }
}

impl<Scope: WorkScope, Func> IntoWorkerTimerOneshotCallback<Scope, (Scope::Payload, Time)> for Func
where
    Func: FnMut(&mut Scope::Payload, Time) + 'static + Send,
{
    fn into_worker_timer_oneshot_callback(mut self) -> AnyTimerCallback<Scope> {
        AnyTimerCallback::OneShot(Box::new(move |payload, t| {
            self(payload, t.handle.clock.now())
        }))
    }
}

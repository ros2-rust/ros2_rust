use crate::{AnyTimerCallback, Node, Time, Timer};

/// This trait is used to create timer callbacks for repeating timers in a Node.
pub trait IntoNodeTimerRepeatingCallback<Args>: 'static + Send {
    /// Convert a suitable object into a repeating timer callback for the node scope
    fn into_node_timer_repeating_callback(self) -> AnyTimerCallback<Node>;
}

impl<Func> IntoNodeTimerRepeatingCallback<()> for Func
where
    Func: FnMut() + 'static + Send,
{
    fn into_node_timer_repeating_callback(mut self) -> AnyTimerCallback<Node> {
        AnyTimerCallback::Repeating(Box::new(move |_, _| self()))
    }
}

impl<Func> IntoNodeTimerRepeatingCallback<Timer> for Func
where
    Func: FnMut(&Timer) + 'static + Send,
{
    fn into_node_timer_repeating_callback(mut self) -> AnyTimerCallback<Node> {
        AnyTimerCallback::Repeating(Box::new(move |_, t| self(t)))
    }
}

impl<Func> IntoNodeTimerRepeatingCallback<Time> for Func
where
    Func: FnMut(Time) + 'static + Send,
{
    fn into_node_timer_repeating_callback(mut self) -> AnyTimerCallback<Node> {
        AnyTimerCallback::Repeating(Box::new(move |_, t| self(t.handle.clock.now())))
    }
}

/// This trait is used to create timer callbacks for one-shot timers in a Node.
pub trait IntoNodeTimerOneshotCallback<Args>: 'static + Send {
    /// Convert a suitable object into a one-shot timer callback for a node scope
    fn into_node_timer_oneshot_callback(self) -> AnyTimerCallback<Node>;
}

impl<Func> IntoNodeTimerOneshotCallback<()> for Func
where
    Func: FnOnce() + 'static + Send,
{
    fn into_node_timer_oneshot_callback(self) -> AnyTimerCallback<Node> {
        AnyTimerCallback::OneShot(Box::new(move |_, _| self()))
    }
}

impl<Func> IntoNodeTimerOneshotCallback<Timer> for Func
where
    Func: FnOnce(&Timer) + 'static + Send,
{
    fn into_node_timer_oneshot_callback(self) -> AnyTimerCallback<Node> {
        AnyTimerCallback::OneShot(Box::new(move |_, t| self(t)))
    }
}

impl<Func> IntoNodeTimerOneshotCallback<Time> for Func
where
    Func: FnOnce(Time) + 'static + Send,
{
    fn into_node_timer_oneshot_callback(self) -> AnyTimerCallback<Node> {
        AnyTimerCallback::OneShot(Box::new(move |_, t| self(t.handle.clock.now())))
    }
}

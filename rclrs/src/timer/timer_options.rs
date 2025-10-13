use std::time::Duration;

use crate::{Clock, NodeState};

/// Options for creating a [`Timer`][crate::Timer].
///
/// The trait [`IntoTimerOptions`] can implicitly convert a single [`Duration`]
/// into `TimerOptions`.
#[derive(Debug, Clone, Copy)]
#[non_exhaustive]
pub struct TimerOptions<'a> {
    /// The period of the timer's interval
    pub period: Duration,
    /// The clock that the timer will reference for its intervals
    pub clock: TimerClock<'a>,
}

impl TimerOptions<'_> {
    /// Make a new timer with a certain interval period, with all other options
    /// as default.
    pub fn new(period: Duration) -> Self {
        Self {
            period,
            clock: TimerClock::default(),
        }
    }
}

/// Trait to implicitly convert a suitable object into [`TimerOptions`].
pub trait IntoTimerOptions<'a>: Sized {
    /// Convert a suitable object into [`TimerOptions`]. This can be used on
    /// [`Duration`] or [`TimerOptions`] itself.
    fn into_timer_options(self) -> TimerOptions<'a>;

    /// Use [`std::time::Instant`] for the timer. This the default so you
    /// shouldn't generally have to call this.
    fn steady_time(self) -> TimerOptions<'a> {
        let mut options = self.into_timer_options();
        options.clock = TimerClock::SteadyTime;
        options
    }

    /// Use [`std::time::SystemTime`] for the timer
    fn system_time(self) -> TimerOptions<'a> {
        let mut options = self.into_timer_options();
        options.clock = TimerClock::SystemTime;
        options
    }

    /// Use the node clock for the timer
    fn node_time(self) -> TimerOptions<'a> {
        let mut options = self.into_timer_options();
        options.clock = TimerClock::NodeTime;
        options
    }

    /// Use a specific clock for the timer
    fn clock(self, clock: &'a Clock) -> TimerOptions<'a> {
        let mut options = self.into_timer_options();
        options.clock = TimerClock::Clock(clock);
        options
    }
}

/// This parameter can specify a type of clock for a timer to use
#[derive(Debug, Default, Clone, Copy)]
pub enum TimerClock<'a> {
    /// Use [`std::time::Instant`] for tracking time
    #[default]
    SteadyTime,
    /// Use [`std::time::SystemTime`] for tracking time
    SystemTime,
    /// Use the parent node's clock for tracking time
    NodeTime,
    /// Use a specific clock for tracking time
    Clock(&'a Clock),
}

impl TimerClock<'_> {
    /// Check if node time has been selected for the timer's clock.
    pub fn is_node_time(&self) -> bool {
        matches!(self, Self::NodeTime)
    }

    pub(crate) fn as_clock(&self, node: &NodeState) -> Clock {
        match self {
            TimerClock::SteadyTime => Clock::steady(),
            TimerClock::SystemTime => Clock::system(),
            TimerClock::NodeTime => node.get_clock(),
            TimerClock::Clock(clock) => (*clock).clone(),
        }
    }
}

impl<'a> IntoTimerOptions<'a> for TimerOptions<'a> {
    fn into_timer_options(self) -> TimerOptions<'a> {
        self
    }
}

impl<'a> IntoTimerOptions<'a> for Duration {
    fn into_timer_options(self) -> TimerOptions<'a> {
        TimerOptions::new(self)
    }
}

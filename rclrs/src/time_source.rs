use crate::clock::ClockSource;
use crate::{Node, ParameterValue, QoSProfile, Subscription, QOS_PROFILE_CLOCK};
use rosgraph_msgs::msg::Clock as ClockMsg;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, Weak};

/// Time source for a node that drives the attached clocks.
/// If the node's `use_sim_time` parameter is set to `true`, the `TimeSource` will subscribe
/// to the `/clock` topic and drive the attached clocks
pub struct TimeSource {
    _node: Weak<Node>,
    _clocks: Arc<Mutex<Vec<ClockSource>>>,
    _clock_qos: QoSProfile,
    // TODO(luca) implement clock threads, for now will run in main thread
    _use_clock_thread: bool,
    // TODO(luca) Update this with parameter callbacks for use_sim_time
    _ros_time_active: Arc<AtomicBool>,
    _clock_subscription: Option<Arc<Subscription<ClockMsg>>>,
    _last_time_msg: Arc<Mutex<Option<ClockMsg>>>,
}

/// A builder for creating a [`TimeSource`][1].
///
/// The builder pattern allows selectively setting some fields, and leaving all others at their default values.
/// This struct instance can be created via [`TimeSource::builder()`][2].
///
/// The default values for optional fields are:
/// - `clock_qos: QOS_PROFILE_CLOCK`[3]
/// - `use_clock_thread: true`
///
///
/// [1]: crate::TimeSource
/// [2]: crate::TimeSource::builder
/// [3]: crate::QOS_PROFILE_CLOCK
pub struct TimeSourceBuilder {
    node: Weak<Node>,
    clock_qos: QoSProfile,
    use_clock_thread: bool,
    clock_source: ClockSource,
}

impl TimeSourceBuilder {
    /// Creates a builder for a time source that drives the given clock for the given node.
    pub fn new(node: Arc<Node>, clock_source: ClockSource) -> Self {
        Self {
            node: Arc::downgrade(&node),
            clock_qos: QOS_PROFILE_CLOCK,
            use_clock_thread: true,
            clock_source,
        }
    }

    /// Sets the QoS for the `/clock` topic.
    pub fn clock_qos(mut self, clock_qos: QoSProfile) -> Self {
        self.clock_qos = clock_qos;
        self
    }

    /// Sets use_clock_thread.
    ///
    /// If set to `true`, the clock callbacks will run in a separate thread.
    /// NOTE: Currently unimplemented
    pub fn use_clock_thread(mut self, use_clock_thread: bool) -> Self {
        self.use_clock_thread = use_clock_thread;
        self
    }

    /// Builds the `TimeSource` and attaches the provided `Node` and `Clock`.
    pub fn build(self) -> TimeSource {
        let mut source = TimeSource {
            _node: Weak::new(),
            _clocks: Arc::new(Mutex::new(vec![])),
            _clock_qos: self.clock_qos,
            _use_clock_thread: self.use_clock_thread,
            _ros_time_active: Arc::new(AtomicBool::new(false)),
            _clock_subscription: None,
            _last_time_msg: Arc::new(Mutex::new(None)),
        };
        source.attach_clock(self.clock_source);
        source.attach_node(self.node);
        source
    }
}

impl TimeSource {
    /// Creates a new `TimeSource` with default parameters.
    pub fn new(node: Arc<Node>, clock: ClockSource) -> Self {
        TimeSourceBuilder::new(node, clock).build()
    }

    /// Creates a new `TimeSourceBuilder` with default parameters.
    pub fn builder(node: Arc<Node>, clock: ClockSource) -> TimeSourceBuilder {
        TimeSourceBuilder::new(node, clock)
    }

    /// Attaches the given clock to the `TimeSource`, enabling the `TimeSource` to control it.
    pub fn attach_clock(&self, clock: ClockSource) {
        if let Some(last_msg) = self._last_time_msg.lock().unwrap().as_ref() {
            let nanoseconds: i64 =
                (last_msg.clock.sec as i64 * 1_000_000_000) + last_msg.clock.nanosec as i64;
            Self::update_clock(&clock, nanoseconds);
        }
        let mut clocks = self._clocks.lock().unwrap();
        if clocks.iter().all(|c| c != &clock) {
            clocks.push(clock);
        }
    }

    /// Detaches the given clock from the `TimeSource`.
    // TODO(luca) should we return a result to denote whether the clock was removed?
    pub fn detach_clock(&self, clock: &ClockSource) {
        self._clocks.lock().unwrap().retain(|c| c != clock);
    }

    /// Attaches the given node to to the `TimeSource`, using its interface to read the
    /// `use_sim_time` parameter and create the clock subscription.
    pub fn attach_node(&mut self, node: Weak<Node>) {
        self._node = node;

        // TODO(luca) register a parameter callback
        if let Some(sim_param) = self
            ._node
            .upgrade()
            .and_then(|n| n.get_parameter("use_sim_time"))
        {
            match sim_param {
                ParameterValue::Bool(val) => {
                    self.set_ros_time_enable(val);
                }
                _ => panic!("use_sim_time parameter must be boolean"),
            }
        }
    }

    fn set_ros_time_enable(&mut self, enable: bool) {
        let updated = self
            ._ros_time_active
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |prev| {
                if prev != enable {
                    Some(enable)
                } else {
                    None
                }
            })
            .is_ok();
        if updated {
            for clock in self._clocks.lock().unwrap().iter() {
                clock.set_ros_time_enable(enable);
            }
            self._clock_subscription = enable.then(|| self.create_clock_sub());
        }
    }

    fn update_clock(clock: &ClockSource, nanoseconds: i64) {
        clock.set_ros_time_override(nanoseconds);
    }

    fn update_all_clocks(clocks: &Arc<Mutex<Vec<ClockSource>>>, nanoseconds: i64) {
        let clocks = clocks.lock().unwrap();
        for clock in clocks.iter() {
            Self::update_clock(clock, nanoseconds);
        }
    }

    fn create_clock_sub(&self) -> Arc<Subscription<ClockMsg>> {
        let clocks = self._clocks.clone();
        let last_time_msg = self._last_time_msg.clone();
        // Safe to unwrap since the function will only fail if invalid arguments are provided
        self._node
            .upgrade()
            .unwrap()
            .create_subscription::<ClockMsg, _>("/clock", self._clock_qos, move |msg: ClockMsg| {
                let nanoseconds: i64 =
                    (msg.clock.sec as i64 * 1_000_000_000) + msg.clock.nanosec as i64;
                *last_time_msg.lock().unwrap() = Some(msg);
                Self::update_all_clocks(&clocks, nanoseconds);
            })
            .unwrap()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::create_node;
    use crate::{Clock, Context};

    #[test]
    fn time_source_attach_clock() {
        let node = create_node(&Context::new([]).unwrap(), "test_node").unwrap();
        let (_, source) = Clock::with_source().unwrap();
        let time_source = TimeSource::new(node.clone(), source);
        let (_, source) = Clock::with_source().unwrap();
        // Attaching additional clocks should be OK
        time_source.attach_clock(source);
        // Default clock should be above 0 (use_sim_time is default false)
        assert!(node.get_clock().now().nsec > 0);
        // TODO(luca) an integration test by creating a node and setting its use_sim_time
    }
}

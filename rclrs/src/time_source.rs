use crate::clock::{Clock, ClockSource, ClockType};
use crate::{Node, QoSProfile, RclrsError, Subscription, QOS_PROFILE_CLOCK};
use rosgraph_msgs::msg::Clock as ClockMsg;
use std::sync::{Arc, Mutex, RwLock, Weak};

/// Time source for a node that drives the attached clock.
/// If the node's `use_sim_time` parameter is set to `true`, the `TimeSource` will subscribe
/// to the `/clock` topic and drive the attached clock
pub struct TimeSource {
    _node: Weak<Node>,
    _clock: RwLock<Clock>,
    _clock_source: Arc<Mutex<Option<ClockSource>>>,
    _requested_clock_type: ClockType,
    _clock_qos: QoSProfile,
    // TODO(luca) implement clock threads, for now will run in main thread
    _use_clock_thread: bool,
    _clock_subscription: Option<Arc<Subscription<ClockMsg>>>,
    _last_received_time: Arc<Mutex<Option<i64>>>,
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
    clock_type: ClockType,
}

impl TimeSourceBuilder {
    /// Creates a builder for a time source that drives the given clock for the given node.
    pub fn new(node: Weak<Node>, clock_type: ClockType) -> Self {
        Self {
            node,
            clock_qos: QOS_PROFILE_CLOCK,
            use_clock_thread: true,
            clock_type,
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
    pub fn build(self) -> Result<TimeSource, RclrsError> {
        let clock = match self.clock_type {
            ClockType::RosTime | ClockType::SystemTime => Clock::system(),
            ClockType::SteadyTime => Clock::steady(),
        }?;
        let mut source = TimeSource {
            _node: Weak::new(),
            _clock: RwLock::new(clock),
            _clock_source: Arc::new(Mutex::new(None)),
            _requested_clock_type: self.clock_type,
            _clock_qos: self.clock_qos,
            _use_clock_thread: self.use_clock_thread,
            _clock_subscription: None,
            _last_received_time: Arc::new(Mutex::new(None)),
        };
        source.attach_node(self.node);
        Ok(source)
    }
}

impl TimeSource {
    /// Creates a new `TimeSource` with default parameters.
    pub fn new(node: Weak<Node>, clock_type: ClockType) -> Self {
        TimeSourceBuilder::new(node, clock_type).build().unwrap()
    }

    /// Creates a new `TimeSourceBuilder` with default parameters.
    pub fn builder(node: Weak<Node>, clock_type: ClockType) -> TimeSourceBuilder {
        TimeSourceBuilder::new(node, clock_type)
    }

    /// Returns the clock that this TimeSource is controlling.
    pub fn get_clock(&self) -> Clock {
        self._clock.read().unwrap().clone()
    }

    /// Attaches the given node to to the `TimeSource`, using its interface to read the
    /// `use_sim_time` parameter and create the clock subscription.
    pub fn attach_node(&mut self, node: Weak<Node>) {
        self._node = node;

        // TODO(luca) register a parameter callback, hold the parameter handle
        if let Some(sim_param) = self._node.upgrade().map(|n| {
            n.declare_parameter::<bool>("use_sim_time", false)
                .mandatory()
                .unwrap()
                .get()
        }) {
            self.set_ros_time_enable(sim_param);
        }
    }

    fn set_ros_time_enable(&mut self, enable: bool) {
        if matches!(self._requested_clock_type, ClockType::RosTime) {
            let mut clock = self._clock.write().unwrap();
            if enable && matches!(clock.clock_type(), ClockType::SystemTime) {
                // TODO(luca) remove unwrap here?
                let (new_clock, mut clock_source) = Clock::with_source().unwrap();
                if let Some(last_received_time) = *self._last_received_time.lock().unwrap() {
                    Self::update_clock(&mut clock_source, last_received_time);
                }
                *clock = new_clock;
                *self._clock_source.lock().unwrap() = Some(clock_source);
                self._clock_subscription = Some(self.create_clock_sub());
            }
            if !enable && matches!(clock.clock_type(), ClockType::RosTime) {
                // TODO(luca) remove unwrap here?
                *clock = Clock::system().unwrap();
                *self._clock_source.lock().unwrap() = None;
                self._clock_subscription = None;
            }
        }
    }

    fn update_clock(clock: &mut ClockSource, nanoseconds: i64) {
        clock.set_ros_time_override(nanoseconds);
    }

    fn create_clock_sub(&self) -> Arc<Subscription<ClockMsg>> {
        let clock = self._clock_source.clone();
        let last_received_time = self._last_received_time.clone();
        // Safe to unwrap since the function will only fail if invalid arguments are provided
        self._node
            .upgrade()
            .unwrap()
            .create_subscription::<ClockMsg, _>("/clock", self._clock_qos, move |msg: ClockMsg| {
                let nanoseconds: i64 =
                    (msg.clock.sec as i64 * 1_000_000_000) + msg.clock.nanosec as i64;
                *last_received_time.lock().unwrap() = Some(nanoseconds);
                if let Some(clock) = clock.lock().unwrap().as_mut() {
                    Self::update_clock(clock, nanoseconds);
                }
            })
            .unwrap()
    }
}

#[cfg(test)]
mod tests {
    use crate::{create_node, Context};

    #[test]
    fn time_source_attach_clock() {
        let node = create_node(&Context::new([]).unwrap(), "test_node").unwrap();
        // Default clock should be above 0 (use_sim_time is default false)
        assert!(node.get_clock().now().nsec > 0);
        // TODO(luca) an integration test by creating a node and setting its use_sim_time
    }
}

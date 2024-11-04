use crate::{
    clock::{Clock, ClockSource, ClockType},
    vendor::rosgraph_msgs::msg::Clock as ClockMsg,
    Node, QoSProfile, ReadOnlyParameter, Subscription, QOS_PROFILE_CLOCK,
};
use std::sync::{Arc, Mutex, RwLock, Weak};

/// Time source for a node that drives the attached clock.
/// If the node's `use_sim_time` parameter is set to `true`, the `TimeSource` will subscribe
/// to the `/clock` topic and drive the attached clock
pub(crate) struct TimeSource {
    node: Mutex<Weak<Node>>,
    clock: RwLock<Clock>,
    clock_source: Arc<Mutex<Option<ClockSource>>>,
    requested_clock_type: ClockType,
    clock_qos: QoSProfile,
    clock_subscription: Mutex<Option<Arc<Subscription<ClockMsg>>>>,
    last_received_time: Arc<Mutex<Option<i64>>>,
    // TODO(luca) Make this parameter editable when we have parameter callbacks implemented and can
    // safely change clock type at runtime
    use_sim_time: Mutex<Option<ReadOnlyParameter<bool>>>,
}

/// A builder for creating a [`TimeSource`][1].
///
/// The builder pattern allows selectively setting some fields, and leaving all others at their default values.
/// This struct instance can be created via [`TimeSource::builder()`][2].
///
/// The default values for optional fields are:
/// - `clock_qos: QOS_PROFILE_CLOCK`[3]
///
///
/// [1]: crate::TimeSource
/// [2]: crate::TimeSource::builder
/// [3]: crate::QOS_PROFILE_CLOCK
pub(crate) struct TimeSourceBuilder {
    clock_qos: QoSProfile,
    clock_type: ClockType,
}

impl TimeSourceBuilder {
    /// Creates a builder for a time source that drives the given clock.
    pub(crate) fn new(clock_type: ClockType) -> Self {
        Self {
            clock_qos: QOS_PROFILE_CLOCK,
            clock_type,
        }
    }

    /// Sets the QoS for the `/clock` topic.
    pub(crate) fn clock_qos(mut self, clock_qos: QoSProfile) -> Self {
        self.clock_qos = clock_qos;
        self
    }

    /// Builds the `TimeSource` and attaches the provided `Node` and `Clock`.
    pub(crate) fn build(self) -> TimeSource {
        let clock = match self.clock_type {
            ClockType::RosTime | ClockType::SystemTime => Clock::system(),
            ClockType::SteadyTime => Clock::steady(),
        };
        TimeSource {
            node: Mutex::new(Weak::new()),
            clock: RwLock::new(clock),
            clock_source: Arc::new(Mutex::new(None)),
            requested_clock_type: self.clock_type,
            clock_qos: self.clock_qos,
            clock_subscription: Mutex::new(None),
            last_received_time: Arc::new(Mutex::new(None)),
            use_sim_time: Mutex::new(None),
        }
    }
}

impl TimeSource {
    /// Creates a new `TimeSourceBuilder` with default parameters.
    pub(crate) fn builder(clock_type: ClockType) -> TimeSourceBuilder {
        TimeSourceBuilder::new(clock_type)
    }

    /// Returns the clock that this TimeSource is controlling.
    pub(crate) fn get_clock(&self) -> Clock {
        self.clock.read().unwrap().clone()
    }

    /// Attaches the given node to to the `TimeSource`, using its interface to read the
    /// `use_sim_time` parameter and create the clock subscription.
    pub(crate) fn attach_node(&self, node: &Arc<Node>) {
        // TODO(luca) Make this parameter editable and register a parameter callback
        // that calls set_ros_time(bool) once parameter callbacks are implemented.
        let param = node
            .declare_parameter("use_sim_time")
            .default(false)
            .read_only()
            .unwrap();
        *self.node.lock().unwrap() = Arc::downgrade(node);
        self.set_ros_time_enable(param.get());
        *self.use_sim_time.lock().unwrap() = Some(param);
    }

    fn set_ros_time_enable(&self, enable: bool) {
        if matches!(self.requested_clock_type, ClockType::RosTime) {
            let mut clock = self.clock.write().unwrap();
            if enable && matches!(clock.clock_type(), ClockType::SystemTime) {
                let (new_clock, mut clock_source) = Clock::with_source();
                if let Some(last_received_time) = *self.last_received_time.lock().unwrap() {
                    Self::update_clock(&mut clock_source, last_received_time);
                }
                *clock = new_clock;
                *self.clock_source.lock().unwrap() = Some(clock_source);
                *self.clock_subscription.lock().unwrap() = Some(self.create_clock_sub());
            }
            if !enable && matches!(clock.clock_type(), ClockType::RosTime) {
                *clock = Clock::system();
                *self.clock_source.lock().unwrap() = None;
                *self.clock_subscription.lock().unwrap() = None;
            }
        }
    }

    fn update_clock(clock: &mut ClockSource, nanoseconds: i64) {
        clock.set_ros_time_override(nanoseconds);
    }

    fn create_clock_sub(&self) -> Arc<Subscription<ClockMsg>> {
        let clock = self.clock_source.clone();
        let last_received_time = self.last_received_time.clone();
        // Safe to unwrap since the function will only fail if invalid arguments are provided
        self.node
            .lock()
            .unwrap()
            .upgrade()
            .unwrap()
            .create_subscription::<ClockMsg, _>("/clock", self.clock_qos, move |msg: ClockMsg| {
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
    fn time_source_default_clock() {
        let node = create_node(&Context::new([]).unwrap(), &format!("time_source_test_node_{}", line!())).unwrap();
        // Default clock should be above 0 (use_sim_time is default false)
        assert!(node.get_clock().now().nsec > 0);
    }

    #[test]
    fn time_source_sim_time() {
        let ctx = Context::new([
            String::from("--ros-args"),
            String::from("-p"),
            String::from("use_sim_time:=true"),
        ])
        .unwrap();
        let node = create_node(&ctx, &format!("time_source_test_node_{}", line!())).unwrap();
        // Default sim time value should be 0 (no message received)
        assert_eq!(node.get_clock().now().nsec, 0);
    }
}

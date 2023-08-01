use crate::{
    clock::{Clock, ClockType},
    RclrsError,
};
use crate::{Node, ParameterValue, QoSProfile, Subscription, QOS_PROFILE_CLOCK};
use rosgraph_msgs::msg::Clock as ClockMsg;
use std::fmt;
use std::sync::{Arc, Mutex};

/// Time source for a node that drives the attached clocks.
/// If the node's `use_sim_time` parameter is set to `true`, the `TimeSource` will subscribe
/// to the `/clock` topic and drive the attached clocks
pub struct TimeSource {
    _node: Arc<Node>,
    _clocks: Arc<Mutex<Vec<Arc<Mutex<Clock>>>>>,
    _clock_qos: QoSProfile,
    // TODO(luca) implement clock threads, for now will run in main thread
    _use_clock_thread: bool,
    // TODO(luca) Update this with parameter callbacks for use_sim_time
    _ros_time_active: Arc<Mutex<bool>>,
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
    node: Arc<Node>,
    clock_qos: QoSProfile,
    use_clock_thread: bool,
    clock: Arc<Mutex<Clock>>,
}

impl TimeSourceBuilder {
    /// Creates a builder for a time source that drives the given clock for the given node.
    /// The clock's ClockType must be set to `RosTime` to allow updates based on the `/clock`
    /// topic.
    pub fn new(node: Arc<Node>, clock: Arc<Mutex<Clock>>) -> Self {
        Self {
            node,
            clock_qos: QOS_PROFILE_CLOCK,
            use_clock_thread: true,
            clock,
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
            _node: self.node.clone(),
            _clocks: Arc::new(Mutex::new(vec![])),
            _clock_qos: self.clock_qos,
            _use_clock_thread: self.use_clock_thread,
            _ros_time_active: Arc::new(Mutex::new(false)),
            _clock_subscription: None,
            _last_time_msg: Arc::new(Mutex::new(None)),
        };
        source.attach_clock(self.clock);
        source.attach_node(self.node);
        source
    }
}

#[derive(Debug)]
pub struct ClockMismatchError(ClockType);

impl fmt::Display for ClockMismatchError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "Unexpected clock type {:?}, only RosTime can be used as a time source when use_sim_time is set",
            self.0
        )
    }
}

impl TimeSource {
    /// Creates a new time source with default parameters.
    pub fn new(node: Arc<Node>, clock: Arc<Mutex<Clock>>) -> Self {
        TimeSourceBuilder::new(node, clock).build()
    }

    /// Attaches the given clock to the `TimeSource`, enabling the `TimeSource` to control it.
    pub fn attach_clock(&self, clock: Arc<Mutex<Clock>>) -> Result<(), ClockMismatchError> {
        let clock_type = clock.clone().lock().unwrap().clock_type();
        if !matches!(clock_type, ClockType::RosTime) && *self._ros_time_active.lock().unwrap() {
            return Err(ClockMismatchError(clock_type));
        }
        if let Some(last_msg) = self._last_time_msg.lock().unwrap().clone() {
            let nanoseconds: i64 =
                (last_msg.clock.sec as i64 * 1_000_000_000) + last_msg.clock.nanosec as i64;
            Self::update_clock(&clock, nanoseconds);
        }
        // TODO(luca) this would allow duplicates to be stored in the vector but it seems other
        // client libraries do the same, should we check and no-op if the value exists already?
        self._clocks.lock().unwrap().push(clock);
        Ok(())
    }

    /// Detaches the given clock from the `TimeSource`.
    // TODO(luca) should we return a result to denote whether the clock was removed?
    pub fn detach_clock(&self, clock: Arc<Mutex<Clock>>) {
        self._clocks
            .lock()
            .unwrap()
            .retain(|c| !Arc::ptr_eq(c, &clock));
    }

    /// Attaches the given node to to the `TimeSource`, using its interface to read the
    /// `use_sim_time` parameter and create the clock subscription.
    pub fn attach_node(&mut self, node: Arc<Node>) -> Result<(), RclrsError> {
        self._node = node;
        // TODO*luca) REMOVE THIS
        self.set_ros_time(true)?;
        return Ok(());

        // TODO(luca) register a parameter callback
        if let Some(sim_param) = node.get_parameter("use_sim_time") {
            match sim_param {
                ParameterValue::Bool(val) => {
                    self.set_ros_time(val)?;
                }
                // TODO(luca) more graceful error handling
                _ => panic!("use_sim_time parameter must be boolean"),
            }
        }
        Ok(())
    }

    fn set_ros_time(&mut self, enable: bool) -> Result<(), RclrsError> {
        let mut ros_time_active = self._ros_time_active.lock().unwrap();
        if enable != *ros_time_active {
            *ros_time_active = enable;
            for clock in self._clocks.lock().unwrap().iter() {
                clock.lock().unwrap().set_ros_time(enable);
            }
            if enable {
                self._clock_subscription = Some(self.create_clock_sub()?);
            } else {
                self._clock_subscription = None;
            }
        }
        Ok(())
    }

    fn update_clock(clock: &Arc<Mutex<Clock>>, nanoseconds: i64) {
        let clock = clock.lock().unwrap();
        clock.set_ros_time_override(nanoseconds);
    }

    fn update_all_clocks(clocks: &Arc<Mutex<Vec<Arc<Mutex<Clock>>>>>, nanoseconds: i64) {
        let clocks = clocks.lock().unwrap();
        for clock in clocks.iter() {
            Self::update_clock(clock, nanoseconds);
        }
    }

    fn create_clock_sub(&self) -> Result<Arc<Subscription<ClockMsg>>, RclrsError> {
        let ros_time_active = self._ros_time_active.clone();
        let clocks = self._clocks.clone();
        let last_time_msg = self._last_time_msg.clone();
        self._node.create_subscription::<ClockMsg, _>(
            "/clock",
            self._clock_qos,
            move |msg: ClockMsg| {
                if *ros_time_active.lock().unwrap() {
                    let nanoseconds: i64 =
                        (msg.clock.sec as i64 * 1_000_000_000) + msg.clock.nanosec as i64;
                    *last_time_msg.lock().unwrap() = Some(msg);
                    Self::update_all_clocks(&clocks, nanoseconds);
                }
            },
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::create_node;
    use crate::Context;

    #[test]
    fn time_source_attach_clock() {
        let clock = Arc::new(Mutex::new(Clock::new(ClockType::RosTime).unwrap()));
        let node = create_node(&Context::new([]).unwrap(), "test_node").unwrap();
        let time_source = TimeSource::new(node, clock);
        let clock = Arc::new(Mutex::new(Clock::new(ClockType::RosTime).unwrap()));
        // Attaching additional clocks should be OK
        time_source.attach_clock(clock).unwrap();
    }
}

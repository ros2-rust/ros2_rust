use crate::rcl_bindings::*;
use crate::{error::ToResult, to_rclrs_result, RclrsError};
use crate::{Node, ParameterValue, QoSProfile, Subscription, QOS_PROFILE_CLOCK};
use std::fmt;
use std::sync::{Arc, Mutex};

use rosgraph_msgs::msg::Clock as ClockMsg;

/// Enum to describe clock type. Redefined for readability and to eliminate the uninitialized case
/// from the `rcl_clock_type_t` enum in the binding.
#[derive(Clone, Debug, Copy)]
pub enum ClockType {
    RosTime = 1,
    SystemTime = 2,
    SteadyTime = 3,
}

impl From<ClockType> for rcl_clock_type_t {
    fn from(clock_type: ClockType) -> Self {
        match clock_type {
            ClockType::RosTime => rcl_clock_type_t::RCL_ROS_TIME,
            ClockType::SystemTime => rcl_clock_type_t::RCL_SYSTEM_TIME,
            ClockType::SteadyTime => rcl_clock_type_t::RCL_STEADY_TIME,
        }
    }
}

#[derive(Debug)]
pub struct Time {
    nsec: i64,
    clock_type: ClockType,
}

impl From<Time> for rcl_time_point_t {
    fn from(time: Time) -> Self {
        Self {
            nanoseconds: time.nsec,
            clock_type: time.clock_type.into(),
        }
    }
}

pub struct Clock {
    _type: ClockType,
    _rcl_clock: Arc<Mutex<rcl_clock_t>>,
    // TODO(luca) Implement jump callbacks
}

impl Clock {
    // TODO(luca) proper error handling
    pub fn new(type_: ClockType) -> Result<Self, RclrsError> {
        let mut rcl_clock;
        unsafe {
            // SAFETY: Getting a default value is always safe.
            rcl_clock = Self::init_generic_clock();
            let mut allocator = rcutils_get_default_allocator();
            rcl_clock_init(type_.into(), &mut rcl_clock, &mut allocator).ok()?;
        }
        Ok(Self {
            _type: type_,
            _rcl_clock: Arc::new(Mutex::new(rcl_clock)),
        })
    }

    pub fn rcl_clock(&self) -> Arc<Mutex<rcl_clock_t>> {
        self._rcl_clock.clone()
    }

    pub fn clock_type(&self) -> ClockType {
        self._type
    }

    pub fn set_ros_time(&mut self, enable: bool) {
        let mut clock = self._rcl_clock.lock().unwrap();
        if enable {
            // SAFETY: Safe if clock jump callbacks are not edited, which is guaranteed
            // by the mutex
            unsafe {
                rcl_enable_ros_time_override(&mut *clock);
            }
        } else {
            // SAFETY: Safe if clock jump callbacks are not edited, which is guaranteed
            // by the mutex
            unsafe {
                rcl_disable_ros_time_override(&mut *clock);
            }
        }
    }

    pub fn now(&self) -> Result<Time, RclrsError> {
        let mut clock = self._rcl_clock.lock().unwrap();
        let mut time_point: i64 = 0;
        unsafe {
            // SAFETY: No preconditions for this function
            rcl_clock_get_now(&mut *clock, &mut time_point).ok()?;
        }
        Ok(Time {
            nsec: time_point,
            clock_type: self._type,
        })
    }

    /// Helper function to initialize a default clock, same behavior as `rcl_init_generic_clock`.
    /// Needed because functions that initialize a clock take as an input a mutable reference
    /// to a clock and don't actuall return one, so we need a function to generate one. Doing this
    /// instead of a `Default` implementation allows the function to be private and avoids
    /// exposing a public API to create an invalid clock
    // SAFETY: Getting a default value is always safe.
    unsafe fn init_generic_clock() -> rcl_clock_t {
        let allocator = rcutils_get_default_allocator();
        rcl_clock_t {
            type_: rcl_clock_type_t::RCL_CLOCK_UNINITIALIZED,
            jump_callbacks: std::ptr::null_mut::<rcl_jump_callback_info_t>(),
            num_jump_callbacks: 0,
            get_now: None,
            data: std::ptr::null_mut::<std::os::raw::c_void>(),
            allocator,
        }
    }
}

impl Drop for Clock {
    fn drop(&mut self) {
        // SAFETY: No preconditions for this function
        let rc = unsafe { rcl_clock_fini(&mut *self._rcl_clock.lock().unwrap()) };
        if let Err(e) = to_rclrs_result(rc) {
            panic!("Unable to release Clock. {:?}", e)
        }
    }
}

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_clock_t {}

/// Time source for a node that subscribes to the /clock node, if the use_sim_time parameter is set
pub struct TimeSource {
    _node: Arc<Node>,
    _clocks: Arc<Mutex<Vec<Arc<Mutex<Clock>>>>>,
    _clock_qos: QoSProfile,
    // TODO(luca) implement clock threads, for now will run in main thread
    _use_clock_thread: bool,
    // TODO(luca) Update this with parameter callbacks for use_sim_time
    _ros_time_active: Arc<Mutex<bool>>,
    _clock_subscription: Option<Arc<Subscription<ClockMsg>>>,
    // TODO(luca) add last time message for newly attached clocks initialization
}

pub struct TimeSourceBuilder {
    node: Arc<Node>,
    clock_qos: QoSProfile,
    use_clock_thread: bool,
    clock: Arc<Mutex<Clock>>,
}

impl TimeSourceBuilder {
    pub fn new(node: Arc<Node>, clock: Arc<Mutex<Clock>>) -> Self {
        Self {
            node,
            clock_qos: QOS_PROFILE_CLOCK,
            use_clock_thread: true,
            clock,
        }
    }

    pub fn clock_qos(mut self, clock_qos: QoSProfile) -> Self {
        self.clock_qos = clock_qos;
        self
    }

    pub fn use_clock_thread(mut self, use_clock_thread: bool) -> Self {
        self.use_clock_thread = use_clock_thread;
        self
    }

    pub fn build(self) -> TimeSource {
        let mut source = TimeSource {
            _node: self.node.clone(),
            _clocks: Arc::new(Mutex::new(vec![])),
            _clock_qos: self.clock_qos,
            _use_clock_thread: self.use_clock_thread,
            _ros_time_active: Arc::new(Mutex::new(false)),
            _clock_subscription: None,
        };
        source.attach_clock(self.clock);
        source.attach_node(self.node);
        source
    }
}

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
    pub fn attach_clock(&mut self, clock: Arc<Mutex<Clock>>) -> Result<(), ClockMismatchError> {
        {
            let clock = clock.lock().unwrap();
            let clock_type = clock.clock_type();
            if !matches!(clock_type, ClockType::RosTime) && *self._ros_time_active.lock().unwrap() {
                return Err(ClockMismatchError(clock_type));
            }
        }
        // TODO(luca) this would allow duplicates to be stored in the vector but it seems other
        // client libraries do the same, should we check and no-op if the value exists already?
        self._clocks.lock().unwrap().push(clock);
        Ok(())
    }

    // TODO(luca) should we return a result to denote whether the clock was removed?
    pub fn detach_clock(&mut self, clock: Arc<Mutex<Clock>>) {
        self._clocks
            .lock()
            .unwrap()
            .retain(|c| !Arc::ptr_eq(c, &clock));
    }

    pub fn attach_node(&mut self, node: Arc<Node>) -> Result<(), RclrsError> {
        self._node = node;
        println!("Checking for use sim time");
        // TODO*luca) REMOVE THIS
        self._clock_subscription = Some(self.create_clock_sub()?);
        self.set_ros_time(true);
        return Ok(());

        if let Some(sim_param) = node.get_parameter("use_sim_time") {
            match sim_param {
                ParameterValue::Bool(val) => {
                    if val {
                        println!("use sim time set");
                        self._clock_subscription = Some(self.create_clock_sub()?);
                        self.set_ros_time(true);
                    } else {
                        // TODO(luca) cleanup subscription, clear set_ros_time
                    }
                }
                // TODO(luca) more graceful error handling
                _ => panic!("use_sim_time parameter must be boolean"),
            }
        }
        Ok(())
    }

    fn set_ros_time(&self, enable: bool) {
        *self._ros_time_active.lock().unwrap() = enable;
        for clock in self._clocks.lock().unwrap().iter() {
            clock.lock().unwrap().set_ros_time(enable);
        }
    }

    fn update_all_clocks(clocks: &Arc<Mutex<Vec<Arc<Mutex<Clock>>>>>, nanoseconds: i64) {
        let clocks = clocks.lock().unwrap();
        for clock in clocks.iter() {
            let clock = clock.lock().unwrap().rcl_clock();
            let mut clock = clock.lock().unwrap();
            // SAFETY: Safe if clock jump callbacks are not edited, which is guaranteed
            // by the mutex
            unsafe {
                rcl_set_ros_time_override(&mut *clock, nanoseconds);
            }
        }
    }

    fn create_clock_sub(&self) -> Result<Arc<Subscription<ClockMsg>>, RclrsError> {
        let ros_time_active = self._ros_time_active.clone();
        let clocks = self._clocks.clone();
        self._node.create_subscription::<ClockMsg, _>(
            "/clock",
            self._clock_qos,
            move |msg: ClockMsg| {
                if *ros_time_active.lock().unwrap() {
                    let nanoseconds: i64 =
                        (msg.clock.sec as i64 * 1_000_000_000) + msg.clock.nanosec as i64;
                    Self::update_all_clocks(&clocks, nanoseconds);
                }
            },
        )
    }
}

/*
impl Default for Time {
    fn default() -> Self {
        Self {
            nsec: 0,
            clock_type: ClockType::SystemTime,
        }
    }
}

impl From<crate::vendor::builtin_interfaces::msg::Time> for Time {
    fn from(time_msg: crate::vendor::builtin_interfaces::msg::Time) -> Self {
        Self {
            nsec: (time_msg.sec as i64 * 1_000_000_000) + time_msg.nanosec as i64,
            clock_type: ClockType::RosTime,
        }
    }
}
*/

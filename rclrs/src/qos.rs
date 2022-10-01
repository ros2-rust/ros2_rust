use std::time::Duration;

use crate::rcl_bindings::*;

/// The `HISTORY` DDS QoS policy.
///
/// A subscription internally maintains a queue of messages (called "samples" in DDS) that have not
/// been processed yet by the application, and likewise a publisher internally maintains a queue.
///
/// If the history policy is `KeepAll`, this queue is unbounded, and if it is `KeepLast`, it is
/// bounded and old values are discarded when the queue is overfull.
///
/// The `rmw` layer may not be able to handle very large queue depths, e.g. greater than
/// `i32::MAX`.
/// In this case, the functions taking the QoS profile as an argument will return an error.
///
/// # Compatibility
/// | Publisher | Subscription | Compatible |
/// | -- | -- | -- |
/// | KeepLast | KeepLast | yes |
/// | KeepLast | KeepAll | yes |
/// | KeepAll | KeepLast | yes |
/// | KeepAll | KeepAll | yes |
///
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum QoSHistoryPolicy {
    /// Use the default policy of the RMW layer.
    ///
    /// If the default policy is `KeepAll`, the depth will be ignored.
    SystemDefault {
        /// The length of the publisher/subscription queue.
        depth: u32,
    },
    /// Keep only the `depth` most recent messages.
    KeepLast {
        /// The length of the publisher/subscription queue.
        depth: u32,
    },
    /// Keep all messages, at least until other resource limits are exceeded.
    KeepAll,
}

/// The `RELIABILITY` DDS QoS policy.
///
/// This policy determines whether delivery between a publisher and a subscription will be retried
/// until successful, or whether messages may be lost in a trade off for better performance.
///
/// # Compatibility
/// | Publisher | Subscription | Compatible | Behavior |
/// | -- | -- | -- | -- |
/// | Reliable | Reliable | yes | Reliable |
/// | Reliable | BestEffort | yes | Best effort |
/// | BestEffort | Reliable | no | - |
/// | BestEffort | BestEffort | yes | Best effort |
///
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum QoSReliabilityPolicy {
    /// Use the default policy of the RMW layer.
    SystemDefault = 0,
    /// Guarantee delivery of messages.
    Reliable = 1,
    /// Send messages but do not guarantee delivery.
    BestEffort = 2,
}

/// The `DURABILITY` DDS QoS policy.
///
/// If a subscription is created after some messages have already been published, it is possible
/// for the subscription to receive a number of previously-published messages by using the
/// "transient local" durability kind on both ends. For this, the publisher must still exist when
/// the subscription is created.
///
/// # Compatibility
/// | Publisher | Subscription | Compatible | Behavior |
/// | -- | -- | -- | -- |
/// | TransientLocal | TransientLocal | yes | Deliver old messages to new subscriptions |
/// | TransientLocal | Volatile | yes | Deliver only new messages |
/// | Volatile | TransientLocal | no | - |
/// | Volatile | Volatile | yes | Deliver only new messages |
///
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum QoSDurabilityPolicy {
    /// Use the default policy of the RMW layer.
    SystemDefault = 0,
    /// Re-deliver old messages.
    /// - For publishers: Retain messages for later delivery.
    /// - For subscriptions: Request delivery of old messages.
    TransientLocal = 1,
    /// Do not retain/request old messages.
    Volatile = 2,
}

/// The `LIVELINESS` DDS QoS policy.
///
/// This policy describes a publisher's reporting policy for its alive status.
/// For a subscription, these are its requirements for its topic's publishers.
///
/// # Compatibility
/// | Publisher | Subscription | Compatible |
/// | -- | -- | -- |
/// | Automatic | Automatic | yes |
/// | Automatic | ManualByTopic | no |
/// | ManualByTopic | Automatic | yes |
/// | ManualByTopic | ManualByTopic | yes |
///
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum QoSLivelinessPolicy {
    /// Use the default policy of the RMW layer.
    SystemDefault = 0,
    /// The signal that establishes that a topic is alive comes from the ROS `rmw` layer.
    Automatic = 1,
    /// The signal that establishes that a topic is alive is sent explicitly. Only publishing a message
    /// on the topic or an explicit signal from the application to assert liveliness on the topic
    /// will mark the topic as being alive.
    ManualByTopic = 3,
}

/// A duration that can take two special values: System default and infinite.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum QoSDuration {
    /// This will use the RMW implementation's default value,
    /// which may or may not be infinite.
    SystemDefault,
    /// This will act as an infinite duration.
    Infinite,
    /// A specific duration.
    Custom(Duration),
}

/// A Quality of Service profile.
///
/// See [docs.ros.org][1] on Quality of Service settings in general.
///
/// In the general case, a topic can have multiple publishers and multiple subscriptions, each with
/// an individual QoS profile. For each publisher-subscription pair, messages are only delivered if
/// their QoS profiles are compatible.
///
/// # Example
/// ```
/// # use rclrs::{QoSProfile, QoSHistoryPolicy, QOS_PROFILE_SENSOR_DATA};
/// let qos = QoSProfile {
///     history: QoSHistoryPolicy::KeepLast { depth: 1 },
///     ..QOS_PROFILE_SENSOR_DATA
/// };
/// ```
///
/// [1]: https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct QoSProfile {
    /// The history policy.
    pub history: QoSHistoryPolicy,
    /// The reliability policy.
    pub reliability: QoSReliabilityPolicy,
    /// The durability policy.
    pub durability: QoSDurabilityPolicy,
    /// The period at which messages are expected to be sent/received.
    ///
    /// If this is `Infinite`, messages never miss a deadline expectation.
    pub deadline: QoSDuration,
    /// The age at which messages are considered expired and no longer valid.
    ///
    /// If this is `Infinite`, messages do not expire.
    pub lifespan: QoSDuration,
    /// The liveliness policy.
    pub liveliness: QoSLivelinessPolicy,
    /// The time within which the RMW publisher must show that it is alive.
    ///
    /// If this is `Infinite`, liveliness is not enforced.
    pub liveliness_lease_duration: QoSDuration,
    /// If true, any ROS specific namespacing conventions will be circumvented.
    ///
    /// In the case of DDS and topics, for example, this means the typical
    /// ROS specific prefix of `rt` would not be applied as described [here][1].
    ///
    /// This might be useful when trying to directly connect a native DDS topic
    /// with a ROS 2 topic.
    ///
    /// [1]: http://design.ros2.org/articles/topic_and_service_names.html#ros-specific-namespace-prefix
    pub avoid_ros_namespace_conventions: bool,
}

impl From<QoSProfile> for rmw_qos_profile_t {
    fn from(qos: QoSProfile) -> Self {
        Self {
            history: qos.history.into(),
            depth: match qos.history {
                QoSHistoryPolicy::SystemDefault { depth } => depth as usize,
                QoSHistoryPolicy::KeepLast { depth } => depth as usize,
                QoSHistoryPolicy::KeepAll => 0,
            },
            reliability: qos.reliability.into(),
            durability: qos.durability.into(),
            deadline: qos.deadline.into(),
            lifespan: qos.lifespan.into(),
            liveliness: qos.liveliness.into(),
            liveliness_lease_duration: qos.liveliness_lease_duration.into(),
            avoid_ros_namespace_conventions: qos.avoid_ros_namespace_conventions,
        }
    }
}

impl From<QoSHistoryPolicy> for rmw_qos_history_policy_t {
    fn from(policy: QoSHistoryPolicy) -> Self {
        match policy {
            QoSHistoryPolicy::SystemDefault { .. } => {
                rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT
            }
            QoSHistoryPolicy::KeepLast { .. } => {
                rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST
            }
            QoSHistoryPolicy::KeepAll => rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_ALL,
        }
    }
}

impl From<QoSReliabilityPolicy> for rmw_qos_reliability_policy_t {
    fn from(policy: QoSReliabilityPolicy) -> Self {
        match policy {
            QoSReliabilityPolicy::SystemDefault => {
                rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT
            }
            QoSReliabilityPolicy::Reliable => {
                rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE
            }
            QoSReliabilityPolicy::BestEffort => {
                rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
            }
        }
    }
}

impl From<QoSDurabilityPolicy> for rmw_qos_durability_policy_t {
    fn from(policy: QoSDurabilityPolicy) -> Self {
        match policy {
            QoSDurabilityPolicy::SystemDefault => {
                rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT
            }
            QoSDurabilityPolicy::TransientLocal => {
                rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
            }
            QoSDurabilityPolicy::Volatile => {
                rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE
            }
        }
    }
}

impl From<QoSLivelinessPolicy> for rmw_qos_liveliness_policy_t {
    fn from(policy: QoSLivelinessPolicy) -> Self {
        match policy {
            QoSLivelinessPolicy::SystemDefault => {
                rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT
            }
            QoSLivelinessPolicy::Automatic => {
                rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
            }
            QoSLivelinessPolicy::ManualByTopic => {
                rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC
            }
        }
    }
}

impl From<QoSDuration> for rmw_time_t {
    fn from(duration: QoSDuration) -> Self {
        match duration {
            QoSDuration::Custom(dt) => {
                let nonzero_dt = dt.max(Duration::from_nanos(1));
                Self {
                    sec: nonzero_dt.as_secs(),
                    nsec: u64::from(nonzero_dt.subsec_nanos()),
                }
            }
            // See RMW_DURATION_DEFAULT
            QoSDuration::SystemDefault => Self { sec: 0, nsec: 0 },
            // See RMW_DURATION_INFINITE
            QoSDuration::Infinite => Self {
                sec: 9223372036,
                nsec: 854775807,
            },
        }
    }
}

/// Equivalent to `rmw_qos_profile_sensor_data` from the [`rmw` package][1].
///
/// [1]: https://github.com/ros2/rmw/blob/master/rmw/include/rmw/qos_profiles.h
pub const QOS_PROFILE_SENSOR_DATA: QoSProfile = QoSProfile {
    history: QoSHistoryPolicy::KeepLast { depth: 5 },
    reliability: QoSReliabilityPolicy::BestEffort,
    durability: QoSDurabilityPolicy::Volatile,
    deadline: QoSDuration::SystemDefault,
    lifespan: QoSDuration::SystemDefault,
    liveliness: QoSLivelinessPolicy::SystemDefault,
    liveliness_lease_duration: QoSDuration::SystemDefault,
    avoid_ros_namespace_conventions: false,
};

/// Equivalent to `rmw_qos_profile_parameters` from the [`rmw` package][1].
///
/// [1]: https://github.com/ros2/rmw/blob/master/rmw/include/rmw/qos_profiles.h
pub const QOS_PROFILE_PARAMETERS: QoSProfile = QoSProfile {
    history: QoSHistoryPolicy::KeepLast { depth: 1000 },
    reliability: QoSReliabilityPolicy::Reliable,
    durability: QoSDurabilityPolicy::Volatile,
    deadline: QoSDuration::SystemDefault,
    lifespan: QoSDuration::SystemDefault,
    liveliness: QoSLivelinessPolicy::SystemDefault,
    liveliness_lease_duration: QoSDuration::SystemDefault,
    avoid_ros_namespace_conventions: false,
};

/// Equivalent to `rmw_qos_profile_default` from the [`rmw` package][1].
///
/// [1]: https://github.com/ros2/rmw/blob/master/rmw/include/rmw/qos_profiles.h
pub const QOS_PROFILE_DEFAULT: QoSProfile = QoSProfile {
    history: QoSHistoryPolicy::KeepLast { depth: 10 },
    reliability: QoSReliabilityPolicy::Reliable,
    durability: QoSDurabilityPolicy::Volatile,
    deadline: QoSDuration::SystemDefault,
    lifespan: QoSDuration::SystemDefault,
    liveliness: QoSLivelinessPolicy::SystemDefault,
    liveliness_lease_duration: QoSDuration::SystemDefault,
    avoid_ros_namespace_conventions: false,
};

/// Equivalent to `rmw_qos_profile_services_default` from the [`rmw` package][1].
///
/// [1]: https://github.com/ros2/rmw/blob/master/rmw/include/rmw/qos_profiles.h
pub const QOS_PROFILE_SERVICES_DEFAULT: QoSProfile = QoSProfile {
    history: QoSHistoryPolicy::KeepLast { depth: 10 },
    reliability: QoSReliabilityPolicy::Reliable,
    durability: QoSDurabilityPolicy::Volatile,
    deadline: QoSDuration::SystemDefault,
    lifespan: QoSDuration::SystemDefault,
    liveliness: QoSLivelinessPolicy::SystemDefault,
    liveliness_lease_duration: QoSDuration::SystemDefault,
    avoid_ros_namespace_conventions: false,
};

/// Equivalent to `rmw_qos_profile_parameter_events` from the [`rmw` package][1].
///
/// [1]: https://github.com/ros2/rmw/blob/master/rmw/include/rmw/qos_profiles.h
pub const QOS_PROFILE_PARAMETER_EVENTS: QoSProfile = QoSProfile {
    history: QoSHistoryPolicy::KeepAll,
    reliability: QoSReliabilityPolicy::Reliable,
    durability: QoSDurabilityPolicy::Volatile,
    deadline: QoSDuration::SystemDefault,
    lifespan: QoSDuration::SystemDefault,
    liveliness: QoSLivelinessPolicy::SystemDefault,
    liveliness_lease_duration: QoSDuration::SystemDefault,
    avoid_ros_namespace_conventions: false,
};

/// Equivalent to `rmw_qos_profile_system_default` from the [`rmw` package][1].
///
/// [1]: https://github.com/ros2/rmw/blob/master/rmw/include/rmw/qos_profiles.h
pub const QOS_PROFILE_SYSTEM_DEFAULT: QoSProfile = QoSProfile {
    history: QoSHistoryPolicy::SystemDefault { depth: 0 },
    reliability: QoSReliabilityPolicy::SystemDefault,
    durability: QoSDurabilityPolicy::SystemDefault,
    deadline: QoSDuration::SystemDefault,
    lifespan: QoSDuration::SystemDefault,
    liveliness: QoSLivelinessPolicy::SystemDefault,
    liveliness_lease_duration: QoSDuration::SystemDefault,
    avoid_ros_namespace_conventions: false,
};

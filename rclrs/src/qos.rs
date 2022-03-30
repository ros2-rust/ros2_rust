use crate::rcl_bindings::*;

/// The `HISTORY` DDS QoS policy.
///
/// A subscription internally maintains a queue of messages (called "samples" in DDS) that have not been processed yet
/// by the application, and likewise a publisher internally maintains a queue.
///
/// If the history policy is `KeepAll`, this queue is unbounded, and if it is `KeepLast`, it is bounded and old values are discarded when the queue is overfull.
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
    SystemDefault { depth: u32 },
    /// Keep only the `depth` most recent messages.
    KeepLast { depth: u32 },
    /// Keep all messages, at least until other resource limits are exceeded.
    KeepAll,
}

/// The `RELIABILITY` DDS QoS policy.
///
/// This policy determines whether delivery between a publisher and a subscription will be retried
/// until successful, or whether it messages may be lost in a trade off for better performance.
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
    pub history: QoSHistoryPolicy,
    pub reliability: QoSReliabilityPolicy,
    pub durability: QoSDurabilityPolicy,
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
            avoid_ros_namespace_conventions: qos.avoid_ros_namespace_conventions,
            deadline: rmw_time_t { sec: 0, nsec: 0 },
            lifespan: rmw_time_t { sec: 0, nsec: 0 },
            liveliness_lease_duration: rmw_time_t { sec: 0, nsec: 0 },
            liveliness: rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
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

pub const QOS_PROFILE_SENSOR_DATA: QoSProfile = QoSProfile {
    history: QoSHistoryPolicy::KeepLast { depth: 5 },
    reliability: QoSReliabilityPolicy::BestEffort,
    durability: QoSDurabilityPolicy::Volatile,
    avoid_ros_namespace_conventions: false,
};

pub const QOS_PROFILE_PARAMETERS: QoSProfile = QoSProfile {
    history: QoSHistoryPolicy::KeepLast { depth: 1000 },
    reliability: QoSReliabilityPolicy::Reliable,
    durability: QoSDurabilityPolicy::Volatile,
    avoid_ros_namespace_conventions: false,
};

pub const QOS_PROFILE_DEFAULT: QoSProfile = QoSProfile {
    history: QoSHistoryPolicy::KeepLast { depth: 10 },
    reliability: QoSReliabilityPolicy::Reliable,
    durability: QoSDurabilityPolicy::Volatile,
    avoid_ros_namespace_conventions: false,
};

pub const QOS_PROFILE_SERVICES_DEFAULT: QoSProfile = QoSProfile {
    history: QoSHistoryPolicy::KeepLast { depth: 10 },
    reliability: QoSReliabilityPolicy::Reliable,
    durability: QoSDurabilityPolicy::Volatile,
    avoid_ros_namespace_conventions: false,
};

pub const QOS_PROFILE_PARAMETER_EVENTS: QoSProfile = QoSProfile {
    history: QoSHistoryPolicy::KeepAll,
    reliability: QoSReliabilityPolicy::Reliable,
    durability: QoSDurabilityPolicy::Volatile,
    avoid_ros_namespace_conventions: false,
};

pub const QOS_PROFILE_SYSTEM_DEFAULT: QoSProfile = QoSProfile {
    history: QoSHistoryPolicy::SystemDefault { depth: 0 },
    reliability: QoSReliabilityPolicy::SystemDefault,
    durability: QoSDurabilityPolicy::SystemDefault,
    avoid_ros_namespace_conventions: false,
};

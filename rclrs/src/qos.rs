use crate::rcl_bindings::*;

pub enum QoSReliabilityPolicy {
    SystemDefault = 0,
    Reliable = 1,
    BestEffort = 2,
}

pub enum QoSHistoryPolicy {
    SystemDefault = 0,
    KeepLast = 1,
    KeepAll = 2,
}

pub enum QoSDurabilityPolicy {
    SystemDefault = 0,
    TransientLocal = 1,
    Volatile = 2,
}

pub struct QoSProfile {
    pub history: QoSHistoryPolicy,
    pub depth: isize,
    pub reliability: QoSReliabilityPolicy,
    pub durability: QoSDurabilityPolicy,
    pub avoid_ros_namespace_conventions: bool,
}

pub const QOS_PROFILE_SENSOR_DATA: QoSProfile = QoSProfile {
    history: QoSHistoryPolicy::KeepLast,
    depth: 5,
    reliability: QoSReliabilityPolicy::BestEffort,
    durability: QoSDurabilityPolicy::Volatile,
    avoid_ros_namespace_conventions: false,
};

pub const QOS_PROFILE_PARAMETERS: QoSProfile = QoSProfile {
    history: QoSHistoryPolicy::KeepLast,
    depth: 1000,
    reliability: QoSReliabilityPolicy::Reliable,
    durability: QoSDurabilityPolicy::Volatile,
    avoid_ros_namespace_conventions: false,
};

pub const QOS_PROFILE_DEFAULT: QoSProfile = QoSProfile {
    history: QoSHistoryPolicy::KeepLast,
    depth: 10,
    reliability: QoSReliabilityPolicy::Reliable,
    durability: QoSDurabilityPolicy::Volatile,
    avoid_ros_namespace_conventions: false,
};

pub const QOS_PROFILE_SERVICES_DEFAULT: QoSProfile = QoSProfile {
    history: QoSHistoryPolicy::KeepLast,
    depth: 10,
    reliability: QoSReliabilityPolicy::Reliable,
    durability: QoSDurabilityPolicy::Volatile,
    avoid_ros_namespace_conventions: false,
};

pub const QOS_PROFILE_PARAMETER_EVENTS: QoSProfile = QoSProfile {
    history: QoSHistoryPolicy::KeepAll,
    depth: 1000,
    reliability: QoSReliabilityPolicy::Reliable,
    durability: QoSDurabilityPolicy::Volatile,
    avoid_ros_namespace_conventions: false,
};

pub const SYSTEM_DEFAULT: isize = 0;

pub const QOS_PROFILE_SYSTEM_DEFAULT: QoSProfile = QoSProfile {
    history: QoSHistoryPolicy::SystemDefault,
    depth: SYSTEM_DEFAULT,
    reliability: QoSReliabilityPolicy::SystemDefault,
    durability: QoSDurabilityPolicy::SystemDefault,
    avoid_ros_namespace_conventions: false,
};

impl From<QoSProfile> for rmw_qos_profile_t {
    fn from(qos: QoSProfile) -> Self {
        Self {
            history: qos.history.into(),
            depth: qos.depth as usize,
            reliability: qos.reliability.into(),
            durability: qos.durability.into(),
            avoid_ros_namespace_conventions: qos.avoid_ros_namespace_conventions,
            deadline: rmw_time_t { sec: 0, nsec: 0 },
            lifespan: rmw_time_t { sec: 0, nsec: 0 },
            liveliness_lease_duration: rmw_time_t { sec: 0, nsec: 0},
            liveliness: rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
        }
    }
}

impl From<QoSHistoryPolicy> for rmw_qos_history_policy_t {
    fn from(policy: QoSHistoryPolicy) -> Self {
        match policy {
            QoSHistoryPolicy::SystemDefault => {
                rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT
            }
            QoSHistoryPolicy::KeepLast => {
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

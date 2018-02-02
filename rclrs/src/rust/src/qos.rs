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

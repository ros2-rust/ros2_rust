use std::time::SystemTime;

use crate::{rcl_bindings::*, timestamp_to_system_time};

/// Information about an incoming service request.
#[derive(Debug, Clone, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct ServiceInfo {
    /// Time when the message was published by the publisher.
    ///
    /// The `rmw` layer does not specify the exact point at which the RMW implementation
    /// must take the timestamp, but it should be taken consistently at the same point in the
    /// process of publishing a message.
    pub source_timestamp: Option<SystemTime>,
    /// Time when the message was received by the service node.
    ///
    /// The `rmw` layer does not specify the exact point at which the RMW implementation
    /// must take the timestamp, but it should be taken consistently at the same point in the
    /// process of receiving a message.
    pub received_timestamp: Option<SystemTime>,
    /// Unique identifier for the request.
    pub request_id: RequestId,
}

impl ServiceInfo {
    pub(crate) fn from_rmw_service_info(rmw_service_info: &rmw_service_info_t) -> Self {
        Self {
            source_timestamp: timestamp_to_system_time(rmw_service_info.source_timestamp),
            received_timestamp: timestamp_to_system_time(rmw_service_info.received_timestamp),
            request_id: RequestId::from_rmw_request_id(&rmw_service_info.request_id),
        }
    }

    pub(crate) fn zero_initialized_rmw() -> rmw_service_info_t {
        rmw_service_info_t {
            source_timestamp: 0,
            received_timestamp: 0,
            request_id: RequestId::zero_initialized_rmw(),
        }
    }
}

/// Unique identifier for a service request.
///
/// Individually each field in the `RequestId` may be repeated across different
/// requests, but the combination of both values will be unique per request.
#[derive(Debug, Clone, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct RequestId {
    /// A globally unique identifier for the writer of the request.
    #[cfg(ros_distro = "humble")]
    pub writer_guid: [i8; 16usize],

    /// A globally unique identifier for the writer of the request.
    #[cfg(not(ros_distro = "humble"))]
    pub writer_guid: [u8; 16usize],

    /// A number assigned to the request which is unique for the writer who
    /// wrote the request.
    pub sequence_number: i64,
}

impl RequestId {
    pub(crate) fn from_rmw_request_id(rmw_request_id: &rmw_request_id_t) -> Self {
        Self {
            writer_guid: rmw_request_id.writer_guid,
            sequence_number: rmw_request_id.sequence_number,
        }
    }

    pub(crate) fn zero_initialized_rmw() -> rmw_request_id_t {
        rmw_request_id_t {
            writer_guid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            sequence_number: 0,
        }
    }
}

impl rmw_service_info_t {
    pub(super) fn rmw_request_id(&self) -> rmw_request_id_t {
        rmw_request_id_t {
            writer_guid: self.request_id.writer_guid,
            sequence_number: self.request_id.sequence_number,
        }
    }
}

#![cfg(not(ros_distro = "humble"))]

use ros_env::rcl_interfaces::{msg::rmw::*, srv::rmw::*};

use crate::{
    error::ToResult,
    rcl_bindings::{rcutils_logging_get_logger_level, rcutils_logging_set_logger_level},
    IntoPrimitiveOptions, LogSeverity, Node, QoSProfile, RclrsError, Service,
    ENTITY_LIFECYCLE_MUTEX,
};

// The variables only exist to keep a strong reference to the services and are technically unused.
pub(crate) struct LoggingService {
    #[allow(dead_code)]
    get_logger_levels_service: Service<GetLoggerLevels>,
    #[allow(dead_code)]
    set_logger_levels_service: Service<SetLoggerLevels>,
}

impl LoggingService {
    pub(crate) fn new(node: &Node) -> Result<Self, RclrsError> {
        let fqn = node.fully_qualified_name();
        let get_logger_levels_service = node.create_service(
            (fqn.clone() + "/get_logger_levels").qos(QoSProfile::parameter_services_default()),
            |req: GetLoggerLevels_Request| get_logger_levels(req),
        )?;
        let set_logger_levels_service = node.create_service(
            (fqn + "/set_logger_levels").qos(QoSProfile::parameter_services_default()),
            |req: SetLoggerLevels_Request| set_logger_levels(req),
        )?;
        Ok(Self {
            get_logger_levels_service,
            set_logger_levels_service,
        })
    }
}

fn get_logger_levels(req: GetLoggerLevels_Request) -> GetLoggerLevels_Response {
    let levels = req
        .names
        .into_iter()
        .map(|name| {
            let raw_level = {
                let c_name = name.to_cstr();
                let _lifecycle = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
                // SAFETY: The precondition are:
                // - we are passing in a valid CString, borrowed from the request String
                // - not thread-safe, so we lock the global mutex before calling this
                unsafe { rcutils_logging_get_logger_level(c_name.as_ptr()) }
            };
            let level = LogSeverity::try_from(raw_level)
                .map(|severity| severity.as_native() as u32)
                .unwrap_or(u32::from(LoggerLevel::LOG_LEVEL_UNKNOWN));
            LoggerLevel { name, level }
        })
        .collect();
    GetLoggerLevels_Response { levels }
}

fn set_logger_levels(req: SetLoggerLevels_Request) -> SetLoggerLevels_Response {
    let results = req
        .levels
        .into_iter()
        .map(|entry| {
            let severity = match LogSeverity::try_from(entry.level as i32) {
                Ok(severity) => severity,
                Err(err) => {
                    return SetLoggerLevelsResult {
                        successful: false,
                        reason: err.to_string().into(),
                    };
                }
            };
            let result = {
                let c_name = entry.name.to_cstr();
                let _lifecycle = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
                // SAFETY: The preconditions are:
                // - we are passing in a valid CString, borrowed from the request String
                // - the severity level is a known LogSeverity variant, checked above
                // - not thread-safe, so we lock the global mutex before calling this
                unsafe {
                    rcutils_logging_set_logger_level(c_name.as_ptr(), severity.as_native() as i32)
                }
                .ok()
            };
            match result {
                Ok(()) => SetLoggerLevelsResult {
                    successful: true,
                    reason: Default::default(),
                },
                Err(err) => {
                    let reason = match &err {
                        RclrsError::RclError { msg: Some(m), .. }
                        | RclrsError::UnknownRclError { msg: Some(m), .. } => m.to_string(),
                        other => other.to_string(),
                    };
                    SetLoggerLevelsResult {
                        successful: false,
                        reason: reason.into(),
                    }
                }
            }
        })
        .collect();
    SetLoggerLevels_Response { results }
}

#[cfg(test)]
mod tests {
    use std::{
        sync::{
            atomic::{AtomicBool, Ordering},
            Arc,
        },
        time::Duration,
    };

    use ros_env::rcl_interfaces::{msg::rmw::*, srv::rmw::*};
    use rosidl_runtime_rs::{seq, Sequence};

    use crate::*;

    /// Builds an executor with a node that advertises the logger services and
    /// a client node, both under the given namespace.
    fn build_test_pair(ns: &str, enable: bool) -> (Executor, Node, Node) {
        let mut options = NodeOptions::new("node").namespace(ns);
        if enable {
            options = options.enable_logger_service(true);
        }
        let executor = Context::default().create_basic_executor();
        let server = executor.create_node(options).unwrap();
        let client = executor
            .create_node(NodeOptions::new("client").namespace(ns))
            .unwrap();
        (executor, server, client)
    }

    #[test]
    fn services_round_trip_when_enabled() -> Result<(), RclrsError> {
        let (mut executor, _server, client) = build_test_pair("logger_svc_round_trip", true);
        let set_client = client
            .create_client::<SetLoggerLevels>("/logger_svc_round_trip/node/set_logger_levels")?;
        let get_client = client
            .create_client::<GetLoggerLevels>("/logger_svc_round_trip/node/get_logger_levels")?;

        let set_inner = Arc::clone(&set_client);
        let get_inner = Arc::clone(&get_client);
        let clients_ready = client
            .notify_on_graph_change_with_period(Duration::from_millis(1), move || {
                set_inner.service_is_ready().unwrap() && get_inner.service_is_ready().unwrap()
            });
        executor
            .spin(
                SpinOptions::default()
                    .until_promise_resolved(clients_ready)
                    .timeout(Duration::from_secs(2)),
            )
            .first_error()?;

        // Use a name unique to this test to avoid clashes with other tests
        // that mutate the global rcutils logger map.
        let logger_name = "rclrs.logger_svc.round_trip";

        let set_request = SetLoggerLevels_Request {
            levels: seq![LoggerLevel {
                name: logger_name.into(),
                level: u32::from(LoggerLevel::LOG_LEVEL_WARN),
            }],
        };
        let set_done = Arc::new(AtomicBool::new(false));
        let set_done_inner = Arc::clone(&set_done);
        let set_promise = set_client
            .call_then(&set_request, move |response: SetLoggerLevels_Response| {
                assert_eq!(response.results.len(), 1);
                assert!(
                    response.results[0].successful,
                    "Set failed: {}",
                    response.results[0].reason
                );
                set_done_inner.store(true, Ordering::Release);
            })
            .unwrap();
        executor
            .spin(
                SpinOptions::default()
                    .until_promise_resolved(set_promise)
                    .timeout(Duration::from_secs(2)),
            )
            .first_error()?;
        assert!(set_done.load(Ordering::Acquire));

        let get_request = GetLoggerLevels_Request {
            names: seq![logger_name.into()],
        };
        let get_done = Arc::new(AtomicBool::new(false));
        let get_done_inner = Arc::clone(&get_done);
        let get_promise = get_client
            .call_then(&get_request, move |response: GetLoggerLevels_Response| {
                assert_eq!(response.levels.len(), 1);
                assert_eq!(response.levels[0].name.to_string(), logger_name);
                assert_eq!(
                    response.levels[0].level,
                    u32::from(LoggerLevel::LOG_LEVEL_WARN)
                );
                get_done_inner.store(true, Ordering::Release);
            })
            .unwrap();
        executor
            .spin(
                SpinOptions::default()
                    .until_promise_resolved(get_promise)
                    .timeout(Duration::from_secs(2)),
            )
            .first_error()?;
        assert!(get_done.load(Ordering::Acquire));

        Ok(())
    }

    #[test]
    fn unknown_logger_returns_unknown_level() -> Result<(), RclrsError> {
        let (mut executor, _server, client) = build_test_pair("logger_svc_unknown", true);
        let get_client = client
            .create_client::<GetLoggerLevels>("/logger_svc_unknown/node/get_logger_levels")?;

        let get_inner = Arc::clone(&get_client);
        let ready = client
            .notify_on_graph_change_with_period(Duration::from_millis(1), move || {
                get_inner.service_is_ready().unwrap()
            });
        executor
            .spin(
                SpinOptions::default()
                    .until_promise_resolved(ready)
                    .timeout(Duration::from_secs(2)),
            )
            .first_error()?;

        let request = GetLoggerLevels_Request {
            names: seq!["rclrs.logger_svc.never_set".into()],
        };
        let done = Arc::new(AtomicBool::new(false));
        let done_inner = Arc::clone(&done);
        let promise = get_client
            .call_then(&request, move |response: GetLoggerLevels_Response| {
                assert_eq!(response.levels.len(), 1);
                assert_eq!(
                    response.levels[0].level,
                    u32::from(LoggerLevel::LOG_LEVEL_UNKNOWN)
                );
                done_inner.store(true, Ordering::Release);
            })
            .unwrap();
        executor
            .spin(
                SpinOptions::default()
                    .until_promise_resolved(promise)
                    .timeout(Duration::from_secs(2)),
            )
            .first_error()?;
        assert!(done.load(Ordering::Acquire));

        Ok(())
    }

    #[test]
    fn services_absent_by_default() -> Result<(), RclrsError> {
        let (mut executor, server, _client) = build_test_pair("logger_svc_absent", false);

        // Give time for discovery; the services should never appear.
        executor
            .spin(SpinOptions::default().timeout(Duration::from_millis(500)))
            .timeout_ok()
            .first_error()?;

        let names_and_types = server.get_service_names_and_types().unwrap();
        assert!(
            !names_and_types.contains_key("/logger_svc_absent/node/get_logger_levels"),
            "get_logger_levels service should not be advertised by default"
        );
        assert!(
            !names_and_types.contains_key("/logger_svc_absent/node/set_logger_levels"),
            "set_logger_levels service should not be advertised by default"
        );

        Ok(())
    }

    #[test]
    fn invalid_severity_rejected() -> Result<(), RclrsError> {
        let (mut executor, _server, client) = build_test_pair("logger_svc_invalid", true);
        let set_client = client
            .create_client::<SetLoggerLevels>("/logger_svc_invalid/node/set_logger_levels")?;

        let set_inner = Arc::clone(&set_client);
        let ready = client
            .notify_on_graph_change_with_period(Duration::from_millis(1), move || {
                set_inner.service_is_ready().unwrap()
            });
        executor
            .spin(
                SpinOptions::default()
                    .until_promise_resolved(ready)
                    .timeout(Duration::from_secs(2)),
            )
            .first_error()?;

        let request = SetLoggerLevels_Request {
            levels: seq![LoggerLevel {
                name: "rclrs.logger_svc.invalid".into(),
                level: 99,
            }],
        };
        let done = Arc::new(AtomicBool::new(false));
        let done_inner = Arc::clone(&done);
        let promise = set_client
            .call_then(&request, move |response: SetLoggerLevels_Response| {
                assert_eq!(response.results.len(), 1);
                assert!(!response.results[0].successful);
                let reason = response.results[0].reason.to_string();
                assert!(
                    reason.contains("99"),
                    "Expected reason to mention offending value 99, got: {reason}"
                );
                done_inner.store(true, Ordering::Release);
            })
            .unwrap();
        executor
            .spin(
                SpinOptions::default()
                    .until_promise_resolved(promise)
                    .timeout(Duration::from_secs(2)),
            )
            .first_error()?;
        assert!(done.load(Ordering::Acquire));

        Ok(())
    }

    #[test]
    fn unset_severity_accepted() -> Result<(), RclrsError> {
        let (mut executor, _server, client) = build_test_pair("logger_svc_unset", true);
        let set_client =
            client.create_client::<SetLoggerLevels>("/logger_svc_unset/node/set_logger_levels")?;

        let set_inner = Arc::clone(&set_client);
        let ready = client
            .notify_on_graph_change_with_period(Duration::from_millis(1), move || {
                set_inner.service_is_ready().unwrap()
            });
        executor
            .spin(
                SpinOptions::default()
                    .until_promise_resolved(ready)
                    .timeout(Duration::from_secs(2)),
            )
            .first_error()?;

        // Level 0 maps to LogSeverity::Unset, which rcutils treats as
        // "inherit from parent" — a valid configuration.
        let request = SetLoggerLevels_Request {
            levels: seq![LoggerLevel {
                name: "rclrs.logger_svc.unset".into(),
                level: u32::from(LoggerLevel::LOG_LEVEL_UNKNOWN),
            }],
        };
        let done = Arc::new(AtomicBool::new(false));
        let done_inner = Arc::clone(&done);
        let promise = set_client
            .call_then(&request, move |response: SetLoggerLevels_Response| {
                assert_eq!(response.results.len(), 1);
                assert!(
                    response.results[0].successful,
                    "Setting level 0 should succeed; got reason: {}",
                    response.results[0].reason
                );
                done_inner.store(true, Ordering::Release);
            })
            .unwrap();
        executor
            .spin(
                SpinOptions::default()
                    .until_promise_resolved(promise)
                    .timeout(Duration::from_secs(2)),
            )
            .first_error()?;
        assert!(done.load(Ordering::Acquire));

        Ok(())
    }
}

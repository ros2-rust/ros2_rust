#![warn(missing_docs)]
//! Rust client library for ROS 2.
//!
//! For getting started, see the [README][1].
//!
//! [1]: https://github.com/ros2-rust/ros2_rust/blob/main/README.md

mod context;
mod error;
mod node;
mod parameter;
mod qos;
mod wait;

mod rcl_bindings;
mod rcl_utils;

pub use context::*;
pub use error::*;
pub use node::*;
pub use parameter::*;
pub use qos::*;
pub use wait::*;

use crate::rcl_bindings::*;
use crate::rcl_utils::get_rcl_arguments;
use rcl_bindings::rcl_context_is_valid;
use std::ffi::{CStr, CString, NulError};
use std::os::raw::c_char;
use std::time::Duration;

pub use rcl_bindings::rmw_request_id_t;

/// Polls the node for new messages and executes the corresponding callbacks.
///
/// See [`WaitSet::wait`] for the meaning of the `timeout` parameter.
///
/// This may under some circumstances return
/// [`SubscriptionTakeFailed`][1], [`ClientTakeFailed`][1], [`ServiceTakeFailed`][1] when the wait
/// set spuriously wakes up.
/// This can usually be ignored.
///
/// [1]: crate::RclReturnCode
pub fn spin_once(node: &Node, timeout: Option<Duration>) -> Result<(), RclrsError> {
    let live_subscriptions = node.live_subscriptions();
    let live_clients = node.live_clients();
    let live_services = node.live_services();
    let ctx = Context {
        rcl_context_mtx: node.rcl_context_mtx.clone(),
    };
    let mut wait_set = WaitSet::new(
        live_subscriptions.len(),
        0,
        0,
        live_clients.len(),
        live_services.len(),
        0,
        &ctx,
    )?;

    for live_subscription in &live_subscriptions {
        wait_set.add_subscription(live_subscription.clone())?;
    }

    for live_client in &live_clients {
        wait_set.add_client(live_client.clone())?;
    }

    for live_service in &live_services {
        wait_set.add_service(live_service.clone())?;
    }

    let ready_entities = wait_set.wait(timeout)?;

    for ready_subscription in ready_entities.subscriptions {
        ready_subscription.execute()?;
    }

    for ready_client in ready_entities.clients {
        ready_client.execute()?;
    }

    for ready_service in ready_entities.services {
        ready_service.execute()?;
    }

    Ok(())
}

/// Convenience function for calling [`spin_once`] in a loop.
///
/// This function additionally checks that the context is still valid.
pub fn spin(node: &Node) -> Result<(), RclrsError> {
    // The context_is_valid functions exists only to abstract away ROS distro differences
    #[cfg(ros_distro = "foxy")]
    // SAFETY: No preconditions for this function.
    let context_is_valid = || unsafe { rcl_context_is_valid(&mut *node.rcl_context_mtx.lock()) };
    #[cfg(not(ros_distro = "foxy"))]
    // SAFETY: No preconditions for this function.
    let context_is_valid = || unsafe { rcl_context_is_valid(&*node.rcl_context_mtx.lock()) };

    while context_is_valid() {
        match spin_once(node, None) {
            Ok(_)
            | Err(RclrsError::RclError {
                code: RclReturnCode::Timeout,
                ..
            }) => (),
            error => return error,
        }
    }
    Ok(())
}

/// Creates a new node in the empty namespace.
///
/// Convenience function equivalent to [`Node::new`][1].
/// Please see that function's documentation.
///
/// [1]: crate::Node::new
///
/// # Example
/// ```
/// # use rclrs::{Context, RclrsError};
/// let ctx = Context::new([])?;
/// let node = rclrs::create_node(&ctx, "my_node");
/// assert!(node.is_ok());
/// # Ok::<(), RclrsError>(())
/// ```
pub fn create_node(context: &Context, node_name: &str) -> Result<Node, RclrsError> {
    Node::builder(context, node_name).build()
}

/// Creates a [`NodeBuilder`][1].
///
/// Convenience function equivalent to [`NodeBuilder::new()`][2] and [`Node::builder()`][3].
/// Please see that function's documentation.
///
/// [1]: crate::NodeBuilder
/// [2]: crate::NodeBuilder::new
/// [3]: crate::Node::builder
///
/// # Example
/// ```
/// # use rclrs::{Context, RclrsError};
/// let context = Context::new([])?;
/// let node_builder = rclrs::create_node_builder(&context, "my_node");
/// let node = node_builder.build()?;
/// assert_eq!(node.name(), "my_node");
/// # Ok::<(), RclrsError>(())
/// ```
pub fn create_node_builder(context: &Context, node_name: &str) -> NodeBuilder {
    Node::builder(context, node_name)
}

/// Extract non-ROS arguments from program's input arguments.
///
/// `args` is expected to be the input arguments of the program (passed via [`std::env::args()`]),
/// which are expected to contain at least one element - the executable name.
/// According to rcl documentation, ROS arguments are between `--ros-args` and `--` arguments.
/// Everything else is considered as non-ROS and left unparsed. Extracted non-ROS args are
/// returned in the order that they appear (see example below).
///
/// # Example
/// ```
/// let input_args : [String;6] = [
///             "arg1", "--ros-args", "some", "args", "--", "arg2"
///         ].map(|x| x.to_string());
/// let non_ros_args = rclrs::extract_non_ros_args(input_args).unwrap();
/// assert_eq!(non_ros_args.len(), 2);
/// assert_eq!(non_ros_args[0], "arg1");
/// assert_eq!(non_ros_args[1], "arg2");
/// ```
pub fn extract_non_ros_args(
    args: impl IntoIterator<Item = String>,
) -> Result<Vec<String>, RclrsError> {
    // SAFETY: Getting a zero-initialized value is always safe.
    let mut rcl_arguments = unsafe { rcl_get_zero_initialized_arguments() };
    // SAFETY: No preconditions for this function
    let allocator = unsafe { rcutils_get_default_allocator() };

    let (args, cstring_args): (Vec<String>, Vec<Result<CString, RclrsError>>) = args
        .into_iter()
        .map(|arg| {
            let cstring_arg =
                CString::new(arg.as_str()).map_err(|err| RclrsError::StringContainsNul {
                    err,
                    s: arg.clone(),
                });
            (arg, cstring_arg)
        })
        .unzip();
    let cstring_args: Vec<CString> = cstring_args
        .into_iter()
        .collect::<Result<Vec<CString>, RclrsError>>()?;
    // Vector of pointers into cstring_args
    let c_args: Vec<*const c_char> = cstring_args.iter().map(|arg| arg.as_ptr()).collect();

    let argv = if c_args.is_empty() {
        std::ptr::null()
    } else {
        c_args.as_ptr()
    };
    // SAFETY: No preconditions for this function
    let ret = unsafe {
        rcl_parse_arguments(c_args.len() as i32, argv, allocator, &mut rcl_arguments).ok()
    };

    if let Err(err) = ret {
        // SAFETY: No preconditions for this function
        unsafe {
            rcl_arguments_fini(&mut rcl_arguments).ok()?;
        }
        return Err(err);
    }

    let ret = get_rcl_arguments(
        rcl_arguments_get_count_unparsed,
        rcl_arguments_get_unparsed,
        &rcl_arguments,
        &args,
    );
    unsafe {
        // SAFETY: No preconditions for this function
        rcl_arguments_fini(&mut rcl_arguments).ok()?;
    }
    ret
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_non_ros_arguments() -> Result<(), String> {
        // ROS args are expected to be between '--ros-args' and '--'. Everything beside that is 'non-ROS'.
        let input_args: Vec<String> = vec![
            "non-ros1",
            "--ros-args",
            "ros-args",
            "--",
            "non-ros2",
            "non-ros3",
        ]
        .into_iter()
        .map(|x| x.to_string())
        .collect();

        let non_ros_args: Vec<String> = extract_non_ros_args(input_args).unwrap();
        let expected = vec!["non-ros1", "non-ros2", "non-ros3"];

        if non_ros_args.len() != expected.len() {
            return Err(format!(
                "Expected vector size: {}, actual: {}",
                expected.len(),
                non_ros_args.len()
            ));
        } else {
            for i in 0..non_ros_args.len() {
                if non_ros_args[i] != expected[i] {
                    let msg = format!(
                        "Mismatching elements at position: {}. Expected: {}, got: {}",
                        i, expected[i], non_ros_args[i]
                    );
                    return Err(msg);
                }
            }
        }

        Ok(())
    }

    #[test]
    fn test_empty_non_ros_arguments() -> Result<(), RclrsError> {
        let empty_non_ros_args = extract_non_ros_args(vec![])?;
        assert_eq!(empty_non_ros_args.len(), 0);

        Ok(())
    }
}

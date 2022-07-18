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
use crate::rcl_utils::{get_rcl_arguments, UnparsedNonRos};
use rcl_bindings::rcl_context_is_valid;
use std::ffi::{CStr, CString};
use std::os::raw::c_char;
use std::time::Duration;

/// Polls the node for new messages and executes the corresponding callbacks.
///
/// See [`WaitSet::wait`] for the meaning of the `timeout` parameter.
///
/// This may under some circumstances return
/// [`SubscriptionTakeFailed`][1] when the wait set spuriously wakes up.
/// This can usually be ignored.
///
/// [1]: crate::RclReturnCode
pub fn spin_once(node: &Node, timeout: Option<Duration>) -> Result<(), RclrsError> {
    let live_subscriptions = node.live_subscriptions();
    let ctx = Context {
        rcl_context_mtx: node.rcl_context_mtx.clone(),
    };
    let mut wait_set = WaitSet::new(live_subscriptions.len(), &ctx)?;

    for live_subscription in &live_subscriptions {
        wait_set.add_subscription(live_subscription.clone())?;
    }

    let ready_entities = wait_set.wait(timeout)?;
    for ready_subscription in ready_entities.subscriptions {
        ready_subscription.execute()?;
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
/// `args` is expected to be input arguments of the program (passed via [`env::args()`])
/// According to rcl documentation ROS arguments are between `--ros-args` and `--` arguments.
/// Everything beside is considered as non-ROS and left unparsed. Extracted non-ROS args are
/// returned in order as they appear (see example below).
///
/// # Example
/// ```
/// let non_ros_args = rclrs::extract_non_ros_args(vec!["arg1", "--ros-args", "some", "args", "--", "arg2"]);
/// assert_eq!(non_ros_args.len(), 2);
/// assert_eq!(non_ros_args[0], "arg1");
/// assert_eq!(non_ros_args[1], "arg2");
/// ```
pub fn extract_non_ros_args(
    args: impl IntoIterator<Item = String>,
) -> Result<Vec<String>, RclrsError> {
    // SAFETY: No preconditions for this function
    let mut rcl_arguments = unsafe { rcl_get_zero_initialized_arguments() };
    // SAFETY: No preconditions for this function
    let allocator = unsafe { rcutils_get_default_allocator() };

    let cstring_args: Vec<CString> = args
        .into_iter()
        .map(|arg| {
            CString::new(arg.as_str()).map_err(|err| RclrsError::StringContainsNul {
                err,
                s: arg.clone(),
            })
        })
        .collect::<Result<_, _>>()?;
    // Vector of pointers into cstring_args
    let c_args: Vec<*const c_char> = cstring_args.iter().map(|arg| arg.as_ptr()).collect();

    let out_ptr = if c_args.is_empty() {
        std::ptr::null()
    } else {
        c_args.as_ptr()
    };
    // SAFETY: No preconditions for this function
    let ret = unsafe {
        rcl_parse_arguments(c_args.len() as i32, out_ptr, allocator, &mut rcl_arguments).ok()
    };

    if let Err(err) = ret {
        // SAFETY: No preconditions for this function
        unsafe {
            rcl_arguments_fini(&mut rcl_arguments).ok()?;
        }
        return Err(err);
    }

    let args: Vec<String> = c_args
        .into_iter()
        // c_args have been converted from Vec<String> a few lines above, so this call is safe
        // SAFETY: c_args must contain valid CStrings
        .map(|arg| {
            unsafe { CString::from(CStr::from_ptr(arg)) }
                .into_string()
                .unwrap()
        })
        .collect();
    let ret = get_rcl_arguments::<UnparsedNonRos>(&rcl_arguments, &args);
    unsafe {
        // SAFETY: No preconditions for this function
        rcl_arguments_fini(&mut rcl_arguments).ok()?;
    }
    ret
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};
    use std::time::Duration;

    #[test]
    fn test_spin_once() -> Result<(), RclrsError> {
        let context = Context::new(vec![]).unwrap();
        let mut subscriber_node = create_node(&context, "minimal_subscriber")?;
        let num_messages = Arc::new(Mutex::new(0));
        let received_msg = Arc::new(Mutex::new(String::new()));
        let n = num_messages.clone();
        let m = received_msg.clone();
        let publisher = subscriber_node
            .create_publisher::<std_msgs::msg::String>("topic", QOS_PROFILE_DEFAULT)?;
        let _subscription = subscriber_node.create_subscription::<std_msgs::msg::String, _>(
            "topic",
            QOS_PROFILE_DEFAULT,
            move |msg: std_msgs::msg::String| {
                let mut num_messages = n.lock().unwrap();
                let mut received_msg = m.lock().unwrap();

                *num_messages += 1;
                *received_msg = msg.data;
            },
        )?;

        let message = std_msgs::msg::String {
            data: String::from("Hello World"),
        };
        publisher.publish(message)?;
        spin_once(&subscriber_node, Some(Duration::from_millis(500)))?;

        assert_eq!(*num_messages.lock().unwrap(), 1);
        assert_eq!(&*received_msg.lock().unwrap().as_str(), "Hello World");

        Ok(())
    }

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

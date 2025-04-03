#![warn(missing_docs)]
//! Rust client library for ROS 2.
//!
//! For getting started, see the [README][1].
//!
//! [1]: https://github.com/ros2-rust/ros2_rust/blob/main/README.md

mod arguments;
mod client;
mod clock;
mod context;
mod error;
mod executor;
mod logging;
mod node;
mod parameter;
mod publisher;
mod qos;
mod service;
mod subscription;
mod time;
mod time_source;
mod vendor;
mod wait;

#[cfg(test)]
mod test_helpers;

mod rcl_bindings;

#[cfg(feature = "dyn_msg")]
pub mod dynamic_message;

pub use arguments::*;
pub use client::*;
pub use clock::*;
pub use context::*;
pub use error::*;
pub use executor::*;
pub use logging::*;
pub use node::*;
pub use parameter::*;
pub use publisher::*;
pub use qos::*;
pub use rcl_bindings::rmw_request_id_t;
pub use service::*;
pub use subscription::*;
pub use time::*;
use time_source::*;
pub use wait::*;


/// # rclrs - ROS 2 Client Library for Rust
/// 
/// `rclrs` provides Rust bindings and idiomatic wrappers for ROS 2 (Robot Operating System).
/// It enables writing ROS 2 nodes, publishers, subscribers, services and clients in Rust.
///
/// ## Features
///
/// - Native Rust implementation of core ROS 2 concepts
/// - Safe wrappers around rcl C API
/// - Support for publishers, subscribers, services, clients
/// - Async/await support for services and clients
/// - Quality of Service (QoS) configuration
/// - Parameter services
/// - Logging integration
///
/// ## Example
/// Here's a simple publisher-subscriber node:

use std::sync::{Arc, Mutex};
use std_msgs::msg::String as StringMsg;

/// ## Write the basic node structure
/// Since Rust doesn't have inheritance, it's not possible to inherit from `Node` as is common practice in `rclcpp` or `rclpy`.
///
/// Instead, you can store the node as a regular member. Let's use a struct that contains the node, a subscription, and a field for the last message that was received to `main.rs`:
pub struct RepublisherNode {
    _node: Arc<Node>,
    _subscription: Arc<Subscription<StringMsg>>,
    _publisher: Arc<Publisher<StringMsg>>,
    _data: Arc<Mutex<Option<StringMsg>>>,
}

impl RepublisherNode {
    fn _new(executor: &Executor) -> Result<Self, RclrsError> {
        let _node = executor.create_node("republisher")?;
        let _data = Arc::new(Mutex::new(None));
        let data_cb = Arc::clone(&_data);
        let _subscription = _node.create_subscription(
            "in_topic".keep_last(10).transient_local(),
            move |msg: StringMsg| {
                *data_cb.lock().unwrap() = Some(msg);
            },
        )?;
        let _publisher = _node.create_publisher::<std_msgs::msg::String>("out_topic")?;
        Ok(Self {
            _node,
            _subscription,
            _publisher,
            _data,
        })
    }

    fn _republish(&self) -> Result<(), RclrsError> {
        if let Some(s) = &*self._data.lock().unwrap() {
            self._publisher.publish(s)?;
        }
        Ok(())
    }
}

fn _main() -> Result<(), RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let republisher = RepublisherNode::_new(&executor)?;
    std::thread::spawn(move || -> Result<(), RclrsError> {
        loop {
            use std::time::Duration;
            std::thread::sleep(Duration::from_millis(1000));
            republisher._republish()?;
        }
    });
    executor
        .spin(SpinOptions::default())
        .first_error()
        .map_err(|err| err.into())
}
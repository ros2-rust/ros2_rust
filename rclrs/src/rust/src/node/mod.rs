mod publisher;
mod subscription;

use std::rc::Rc;

use qos;

use std;
use std::ffi::CStr;
use std::ffi::CString;

use libc::uintptr_t;
use libc::c_int;
use libc::c_char;
use libc::c_uchar;
use libc::size_t;
use rclrs_common;

use rclrs_common::error::RCLError;
use generate_rcl_error;

#[link(name = "rclrs")]
extern "C" {
    fn rclrs_native_create_publisher_handle(
        publisher_handle: *mut uintptr_t,
        node_handle: uintptr_t,
        type_support_handle: uintptr_t,
        topic: *const c_char,
        qos_policy_history: c_uchar,
        depth: size_t,
        qos_policy_reliability: c_uchar,
        qos_policy_durability: c_uchar,
        avoid_ros_namespace_conventions: c_uchar,
    ) -> c_int;

    fn rclrs_native_create_subscription_handle(
        subscription_handle: *mut uintptr_t,
        node_handle: uintptr_t,
        type_support_handle: uintptr_t,
        topic: *const c_char,
        qos_policy_history: c_uchar,
        depth: size_t,
        qos_policy_reliability: c_uchar,
        qos_policy_durability: c_uchar,
        avoid_ros_namespace_conventions: c_uchar,
    ) -> c_int;
}

pub struct Node {
    handle: uintptr_t,
    name: &'static str,
    pub subscriptions: Vec<Rc<subscription::SubscriptionBase>>,
}

fn create_publisher_handle<T>(
    node_handle: uintptr_t,
    topic: &'static str,
    qos_history: u8,
    qos_depth: usize,
    qos_reliability: u8,
    qos_durability: u8,
    qos_avoid_ros_namespace_conventions: u8,
) -> Result<uintptr_t, RCLError>
where
    T: rclrs_common::traits::MessageDefinition<T>,
{
    let c_string_topic = CString::new(topic).unwrap();
    let mut publisher_handle = 0;
    let ret = unsafe {
        rclrs_native_create_publisher_handle(
            &mut publisher_handle,
            node_handle,
            T::get_type_support(),
            c_string_topic.as_ptr(),
            qos_history,
            qos_depth,
            qos_reliability,
            qos_durability,
            qos_avoid_ros_namespace_conventions,
        )
    };

    match ret {
        0 => Ok(publisher_handle),
        _ => Err(generate_rcl_error(ret)),
    }
}

fn create_subscription_handle<T>(
    node_handle: uintptr_t,
    topic: &'static str,
    qos_history: u8,
    qos_depth: usize,
    qos_reliability: u8,
    qos_durability: u8,
    qos_avoid_ros_namespace_conventions: u8,
) -> Result<uintptr_t, RCLError>
where
    T: rclrs_common::traits::MessageDefinition<T>,
{
    let c_string_topic = CString::new(topic).unwrap();
    let mut subscription_handle = 0;
    let ret = unsafe {
        rclrs_native_create_subscription_handle(
            &mut subscription_handle,
            node_handle,
            T::get_type_support(),
            c_string_topic.as_ptr(),
            qos_history,
            qos_depth,
            qos_reliability,
            qos_durability,
            qos_avoid_ros_namespace_conventions,
        )
    };

    match ret {
        0 => Ok(subscription_handle),
        _ => Err(generate_rcl_error(ret)),
    }
}

impl Node {
    pub fn new(handle: uintptr_t, name: &'static str) -> Node {
        Node {
            handle: handle,
            name: name,
            subscriptions: Vec::new(),
        }
    }

    pub fn create_publisher<T>(
        self: &Self,
        topic: &'static str,
        qos_profile: qos::QoSProfile,
    ) -> publisher::Publisher<T>
    where
        T: rclrs_common::traits::MessageDefinition<T>,
    {
        let publisher_handle = create_publisher_handle::<T>(
            self.handle,
            topic,
            qos_profile.history as u8,
            qos_profile.depth as usize,
            qos_profile.reliability as u8,
            qos_profile.durability as u8,
            qos_profile.avoid_ros_namespace_conventions as u8,
        ).unwrap();
        return publisher::Publisher::new(publisher_handle, topic);
    }

    pub fn create_subscription<T>(
        self: &mut Self,
        topic: &'static str,
        qos_profile: qos::QoSProfile,
        callback: fn(&T),
    ) -> Rc<subscription::Subscription<T>>
    where
        T: rclrs_common::traits::MessageDefinition<T> + std::default::Default,
    {
        let subscription_handle = create_subscription_handle::<T>(
            self.handle,
            topic,
            qos_profile.history as u8,
            qos_profile.depth as usize,
            qos_profile.reliability as u8,
            qos_profile.durability as u8,
            qos_profile.avoid_ros_namespace_conventions as u8,
        ).unwrap();
        let sub = subscription::Subscription::new(subscription_handle, topic, callback);
        let sub_ptr = Rc::new(sub);
        self.subscriptions.push(sub_ptr.clone());
        return sub_ptr;
    }

    pub fn number_of_subscriptions(&self) -> usize {
        return self.subscriptions.len();
    }

    pub fn subscriptions(&self) -> &Vec<Rc<subscription::SubscriptionBase>> {
        return &self.subscriptions;
    }
}

pub mod qos;
mod node;

use std::ffi::CStr;
use std::ffi::CString;

extern crate libc;
extern crate rclrs_common;

use libc::c_char;
use libc::c_long;
use libc::c_int;
use libc::uintptr_t;
use libc::size_t;
use rclrs_common::error::RCLError;
use rclrs_common::error::RCLStatusCode;

pub trait Handle {
    fn handle(&self) -> uintptr_t;
}

#[link(name = "rclrs")]
extern "C" {
    fn rclrs_native_init() -> c_int;
    fn rclrs_native_ok() -> c_int;
    fn rclrs_native_get_error_string_safe() -> *const c_char;
    fn rclrs_native_reset_error() -> ();
    fn rclrs_native_create_node_handle(
        node_handle: *mut uintptr_t,
        name: *const c_char,
        namespace: *const c_char,
    ) -> c_int;

    fn rclrs_native_get_zero_initialized_wait_set() -> uintptr_t;
    fn rclrs_native_destroy_wait_set(wait_set_handle: uintptr_t) -> ();

    fn rclrs_native_wait_set_init(
        wait_set_handle: uintptr_t,
        number_of_subscriptions: size_t,
        number_of_guard_conditions: size_t,
        number_of_timers: size_t,
        number_of_clients: size_t,
        number_of_services: size_t,
    ) -> c_int;

    fn rclrs_native_wait_set_clear_subscriptions(wait_set_handle: uintptr_t) -> c_int;

    fn rclrs_native_wait_set_clear_services(wait_set_handle: uintptr_t) -> c_int;

    fn rclrs_native_wait_set_clear_clients(wait_set_handle: uintptr_t) -> c_int;

    fn rclrs_native_wait_set_add_subscription(
        wait_set_handle: uintptr_t,
        subscription_handle: uintptr_t,
    ) -> c_int;

    fn rclrs_native_wait(wait_set_handle: uintptr_t, timeout: c_long) -> c_int;

    fn rclrs_native_take(subscription_handle: uintptr_t, message_handle: uintptr_t) -> c_int;
}

pub fn init() -> Result<(), RCLError> {
    let ret = unsafe { rclrs_native_init() };
    let rcl_code = RCLStatusCode::from(ret);
    match rcl_code {
        RCLStatusCode::OK => Ok(()),
        _ => Err(generate_rcl_error(ret)),
    }
}


pub fn ok() -> bool {
    unsafe {
        return rclrs_native_ok() != 0;
    }
}

fn generate_rcl_error(ret: i32) -> RCLError {
    let c_error_message = unsafe { CStr::from_ptr(rclrs_native_get_error_string_safe()) };
    let error_message = c_error_message.to_str().unwrap();
    unsafe {
        rclrs_native_reset_error();
    }
    return RCLError {
        code: RCLStatusCode::from(ret),
        message: error_message,
    };
}

fn create_node_handle(name: &str, namespace: &str) -> Result<uintptr_t, RCLError> {
    let c_string_name = CString::new(name).unwrap();
    let c_string_namespace = CString::new(namespace).unwrap();
    let mut node_handle = 0;
    let ret = unsafe {
        rclrs_native_create_node_handle(
            &mut node_handle,
            c_string_name.as_ptr(),
            c_string_namespace.as_ptr(),
        )
    };
    let rcl_code = RCLStatusCode::from(ret);
    match rcl_code {
        RCLStatusCode::OK => Ok(node_handle),
        _ => Err(generate_rcl_error(ret)),
    }
}

pub fn create_node(name: &'static str) -> node::Node {
    return create_node_with_namespace(name, "");
}

pub fn create_node_with_namespace(name: &'static str, namespace: &'static str) -> node::Node {
    let node_handle = create_node_handle(name, namespace).unwrap();
    return node::Node::new(node_handle, name);
}

pub fn get_zero_initialized_wait_set() -> uintptr_t {
    return unsafe { rclrs_native_get_zero_initialized_wait_set() };
}

pub fn destroy_wait_set(wait_set_handle: uintptr_t) -> () {
    unsafe {
        rclrs_native_destroy_wait_set(wait_set_handle);
    };
}

pub fn wait_set_init(
    wait_set_handle: uintptr_t,
    number_of_subscriptions: usize,
    number_of_guard_conditions: usize,
    number_of_timers: usize,
    number_of_clients: usize,
    number_of_services: usize,
) -> Result<(), RCLError> {
    let ret = unsafe {
        rclrs_native_wait_set_init(
            wait_set_handle,
            number_of_subscriptions,
            number_of_guard_conditions,
            number_of_timers,
            number_of_clients,
            number_of_services,
        )
    };
    let rcl_code = RCLStatusCode::from(ret);
    match rcl_code {
        RCLStatusCode::OK => Ok(()),
        _ => Err(generate_rcl_error(ret)),
    }
}

pub fn wait_set_clear_subscriptions(wait_set_handle: uintptr_t) -> Result<(), RCLError> {
    let ret = unsafe { rclrs_native_wait_set_clear_subscriptions(wait_set_handle) };
    let rcl_code = RCLStatusCode::from(ret);
    match rcl_code {
        RCLStatusCode::OK => Ok(()),
        _ => Err(generate_rcl_error(ret)),
    }
}

pub fn wait_set_clear_services(wait_set_handle: uintptr_t) -> Result<(), RCLError> {
    let ret = unsafe { rclrs_native_wait_set_clear_services(wait_set_handle) };
    let rcl_code = RCLStatusCode::from(ret);
    match rcl_code {
        RCLStatusCode::OK => Ok(()),
        _ => Err(generate_rcl_error(ret)),
    }
}

pub fn wait_set_clear_clients(wait_set_handle: uintptr_t) -> Result<(), RCLError> {
    let ret = unsafe { rclrs_native_wait_set_clear_clients(wait_set_handle) };
    let rcl_code = RCLStatusCode::from(ret);
    match rcl_code {
        RCLStatusCode::OK => Ok(()),
        _ => Err(generate_rcl_error(ret)),
    }
}

pub fn wait_set_add_subscription(
    wait_set_handle: uintptr_t,
    subscription_handle: uintptr_t,
) -> Result<(), RCLError> {
    let ret =
        unsafe { rclrs_native_wait_set_add_subscription(wait_set_handle, subscription_handle) };
    let rcl_code = RCLStatusCode::from(ret);
    match rcl_code {
        RCLStatusCode::OK => Ok(()),
        _ => Err(generate_rcl_error(ret)),
    }
}

pub fn wait(wait_set_handle: uintptr_t, timeout: i64) -> Result<(), RCLError> {
    let ret = unsafe { rclrs_native_wait(wait_set_handle, timeout) };
    let rcl_code = RCLStatusCode::from(ret);
    match rcl_code {
        RCLStatusCode::OK => Ok(()),
        _ => Err(generate_rcl_error(ret)),
    }
}

pub fn take(
    subscription_handle: uintptr_t,
    message: &mut rclrs_common::traits::Message,
) -> Result<bool, RCLError> {
    let message_handle = message.get_native_message();
    let ret = unsafe { rclrs_native_take(subscription_handle, message_handle) };
    let rcl_code = RCLStatusCode::from(ret);
    let result = match rcl_code {
        RCLStatusCode::OK => {
            message.read_handle(message_handle);
            Ok(true)
        }
        RCLStatusCode::SubscriptionTakeFailed => Ok(false),
        _ => Err(generate_rcl_error(ret)),
    };
    message.destroy_native_message(message_handle);
    return result;
}

pub fn spin(node: &node::Node) {
    while ok() {
        spin_once(node, 500);
    }
}

pub fn spin_once(node: &node::Node, timeout: i64) -> () {
    let wait_set_handle = get_zero_initialized_wait_set();

    let number_of_subscriptions = node.subscriptions().len();
    let number_of_guard_conditions = 0;
    let number_of_timers = 0;
    let number_of_clients = 0;
    let number_of_services = 0;

    wait_set_init(
        wait_set_handle,
        number_of_subscriptions,
        number_of_guard_conditions,
        number_of_timers,
        number_of_clients,
        number_of_services,
    ).unwrap();

    wait_set_clear_subscriptions(wait_set_handle).unwrap();

    wait_set_clear_services(wait_set_handle).unwrap();

    wait_set_clear_clients(wait_set_handle).unwrap();

    for subscription in node.subscriptions() {
        wait_set_add_subscription(wait_set_handle, subscription.handle()).unwrap();
    }

    wait(wait_set_handle, timeout);

    for subscription in node.subscriptions() {
        let mut message = subscription.create_message();
        let result = take(subscription.handle(), &mut *message).unwrap();
        if result {
            subscription.callback_fn(message);
        }
    }
    destroy_wait_set(wait_set_handle);
}

use crate::error::{RclReturnCode, SubscriberErrorCode, ToResult};
use crate::qos::QoSProfile;
use crate::Node;
use crate::{rcl_bindings::*, RclrsError};

use std::borrow::Borrow;
use std::boxed::Box;
use std::ffi::CString;
use std::marker::PhantomData;
use std::sync::Arc;

use rosidl_runtime_rs::{Message, RmwMessage};

use parking_lot::{Mutex, MutexGuard};

/// Internal struct used by subscriptions.
pub struct SubscriptionHandle {
    handle: Mutex<rcl_subscription_t>,
    node_handle: Arc<Mutex<rcl_node_t>>,
}

impl SubscriptionHandle {
    pub(crate) fn lock(&self) -> MutexGuard<rcl_subscription_t> {
        self.handle.lock()
    }
}

impl Drop for SubscriptionHandle {
    fn drop(&mut self) {
        let handle = self.handle.get_mut();
        let node_handle = &mut *self.node_handle.lock();
        // SAFETY: No preconditions for this function (besides the arguments being valid).
        unsafe {
            rcl_subscription_fini(handle, node_handle);
        }
    }
}

/// Trait to be implemented by concrete [`Subscription`]s.
pub trait SubscriptionBase {
    /// Internal function to get a reference to the `rcl` handle.
    fn handle(&self) -> &SubscriptionHandle;
    /// Tries to take a new message and run the callback with it.
    fn execute(&self) -> Result<(), RclrsError>;
}

/// Struct for receiving messages of type `T`.
///
/// There can be multiple subscriptions for the same topic, in different nodes or the same node.
///
/// Receiving messages requires calling [`spin_once`][1] or [`spin`][2] on the subscription's node.
///
/// When a subscription is created, it may take some time to get "matched" with a corresponding
/// publisher.
///
/// [1]: crate::spin_once
/// [2]: crate::spin
pub struct Subscription<T>
where
    T: Message,
{
    pub(crate) handle: Arc<SubscriptionHandle>,
    /// The callback function that runs when a message was received.
    pub callback: Mutex<Box<dyn FnMut(T) + 'static>>,
    message: PhantomData<T>,
}

impl<T> Subscription<T>
where
    T: Message,
{
    /// Creates a new subscription.
    ///
    /// # Panics
    /// When the topic contains interior null bytes.
    pub fn new<F>(
        node: &Node,
        topic: &str,
        qos: QoSProfile,
        callback: F,
    ) -> Result<Self, RclrsError>
    where
        T: Message,
        F: FnMut(T) + 'static,
    {
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut subscription_handle = unsafe { rcl_get_zero_initialized_subscription() };
        let type_support =
            <T as Message>::RmwMsg::get_type_support() as *const rosidl_message_type_support_t;
        let topic_c_string = CString::new(topic).unwrap();
        let node_handle = &mut *node.handle.lock();

        // SAFETY: No preconditions for this function.
        let mut subscription_options = unsafe { rcl_subscription_get_default_options() };
        subscription_options.qos = qos.into();
        unsafe {
            // SAFETY: The subscription handle is zero-initialized as expected by this function.
            // The node handle is kept alive because it is co-owned by the subscription.
            // The topic name and the options are copied by this function, so they can be dropped
            // afterwards.
            // TODO: type support?
            rcl_subscription_init(
                &mut subscription_handle,
                node_handle,
                type_support,
                topic_c_string.as_ptr(),
                &subscription_options,
            )
            .ok()?;
        }

        let handle = Arc::new(SubscriptionHandle {
            handle: Mutex::new(subscription_handle),
            node_handle: node.handle.clone(),
        });

        Ok(Self {
            handle,
            callback: Mutex::new(Box::new(callback)),
            message: PhantomData,
        })
    }

    /// Fetches a new message.
    ///
    /// When there is no new message, this will return a
    /// [`SubscriptionTakeFailed`][1] wrapped in an [`RclrsError`][2].
    ///
    /// [1]: crate::SubscriberErrorCode
    /// [2]: crate::RclrsError
    //
    // ```text
    // +-------------+
    // | rclrs::take |
    // +------+------+
    //        |
    //        |
    // +------v------+
    // |  rcl_take   |
    // +------+------+
    //        |
    //        |
    // +------v------+
    // |  rmw_take   |
    // +-------------+
    // ```
    pub fn take(&self) -> Result<T, RclrsError> {
        let mut rmw_message = <T as Message>::RmwMsg::default();
        let handle = &mut *self.handle.lock();
        let ret = unsafe {
            // SAFETY: The first two pointers are valid/initialized, and do not need to be valid
            // beyond the function call.
            // The latter two pointers are explicitly allowed to be NULL.
            rcl_take(
                handle,
                &mut rmw_message as *mut <T as Message>::RmwMsg as *mut _,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
            )
        };
        ret.ok()?;
        Ok(T::from_rmw_message(rmw_message))
    }
}

impl<T> SubscriptionBase for Subscription<T>
where
    T: Message,
{
    fn handle(&self) -> &SubscriptionHandle {
        self.handle.borrow()
    }

    fn execute(&self) -> Result<(), RclrsError> {
        let msg = match self.take() {
            Ok(msg) => msg,
            Err(RclrsError {
                code: RclReturnCode::SubscriberError(SubscriberErrorCode::SubscriptionTakeFailed),
                ..
            }) => {
                // Spurious wakeup – this may happen even when a waitset indicated that this
                // subscription was ready, so it shouldn't be an error.
                return Ok(());
            }
            Err(e) => return Err(e),
        };
        (*self.callback.lock())(msg);
        Ok(())
    }
}

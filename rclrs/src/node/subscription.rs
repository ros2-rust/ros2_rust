use crate::error::{RclReturnCode, ToResult};
use crate::{rcl_bindings::*, RclrsError};
use crate::{Node, QoSProfile, WaitSet, Waitable};

use std::boxed::Box;
use std::ffi::CStr;
use std::ffi::CString;
use std::marker::PhantomData;
use std::sync::Arc;

use rosidl_runtime_rs::{Message, RmwMessage};

use parking_lot::Mutex;

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_subscription_t {}

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
    rcl_subscription_mtx: Mutex<rcl_subscription_t>,
    rcl_node_mtx: Arc<Mutex<rcl_node_t>>,
    /// The callback function that runs when a message was received.
    pub callback: Mutex<Box<dyn FnMut(T) + 'static + Send>>,
    message: PhantomData<T>,
}

impl<T: Message> Drop for Subscription<T> {
    fn drop(&mut self) {
        let rcl_subscription = self.rcl_subscription_mtx.get_mut();
        let rcl_node = &mut *self.rcl_node_mtx.lock();
        // SAFETY: No preconditions for this function (besides the arguments being valid).
        unsafe {
            rcl_subscription_fini(rcl_subscription, rcl_node);
        }
    }
}

impl<T> Waitable for Subscription<T>
where
    T: Message,
{
    unsafe fn add_to_wait_set(self: Arc<Self>, wait_set: &mut WaitSet) -> Result<(), RclrsError> {
        // SAFETY: I'm not sure if it's required, but the subscription pointer will remain valid
        // for as long as the wait set exists, because it's stored in self.subscriptions.
        // Passing in a null pointer for the third argument is explicitly allowed.
        rcl_wait_set_add_subscription(
            &mut wait_set.rcl_wait_set,
            &*self.rcl_subscription_mtx.lock(),
            std::ptr::null_mut(),
        )
        .ok()?;
        wait_set.subscriptions.push(self);
        Ok(())
    }

    fn execute(&self) -> Result<(), RclrsError> {
        let msg = match self.take() {
            Ok(msg) => msg,
            Err(RclrsError::RclError {
                code: RclReturnCode::SubscriptionTakeFailed,
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

impl<T> Subscription<T>
where
    T: Message,
{
    /// Creates a new subscription.
    pub fn new<F>(
        node: &Node,
        topic: &str,
        qos: QoSProfile,
        callback: F,
    ) -> Result<Self, RclrsError>
    where
        T: Message,
        F: FnMut(T) + 'static + Send,
    {
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_subscription = unsafe { rcl_get_zero_initialized_subscription() };
        let type_support =
            <T as Message>::RmwMsg::get_type_support() as *const rosidl_message_type_support_t;
        let topic_c_string = CString::new(topic).map_err(|err| RclrsError::StringContainsNul {
            err,
            s: topic.into(),
        })?;
        let rcl_node = &mut *node.rcl_node_mtx.lock();

        // SAFETY: No preconditions for this function.
        let mut subscription_options = unsafe { rcl_subscription_get_default_options() };
        subscription_options.qos = qos.into();
        unsafe {
            // SAFETY: The rcl_subscription is zero-initialized as expected by this function.
            // The rcl_node is kept alive because it is co-owned by the subscription.
            // The topic name and the options are copied by this function, so they can be dropped
            // afterwards.
            // TODO: type support?
            rcl_subscription_init(
                &mut rcl_subscription,
                rcl_node,
                type_support,
                topic_c_string.as_ptr(),
                &subscription_options,
            )
            .ok()?;
        }

        Ok(Self {
            rcl_subscription_mtx: Mutex::new(rcl_subscription),
            rcl_node_mtx: Arc::clone(&node.rcl_node_mtx),
            callback: Mutex::new(Box::new(callback)),
            message: PhantomData,
        })
    }

    /// Returns the topic name of the subscription.
    ///
    /// This returns the topic name after remapping, so it is not necessarily the
    /// topic name which was used when creating the subscription.
    pub fn topic_name(&self) -> String {
        // SAFETY: No preconditions for the function used
        // The unsafe variables get converted to safe types before being returned
        unsafe {
            let raw_topic_pointer =
                rcl_subscription_get_topic_name(&*self.rcl_subscription_mtx.lock());
            CStr::from_ptr(raw_topic_pointer)
                .to_string_lossy()
                .into_owned()
        }
    }

    /// Fetches a new message.
    ///
    /// When there is no new message, this will return a
    /// [`SubscriptionTakeFailed`][1].
    ///
    /// [1]: crate::RclrsError
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
        let rcl_subscription = &mut *self.rcl_subscription_mtx.lock();
        unsafe {
            // SAFETY: The first two pointers are valid/initialized, and do not need to be valid
            // beyond the function call.
            // The latter two pointers are explicitly allowed to be NULL.
            rcl_take(
                rcl_subscription,
                &mut rmw_message as *mut <T as Message>::RmwMsg as *mut _,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
            )
            .ok()?
        };
        Ok(T::from_rmw_message(rmw_message))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{create_node, Context, Subscription, QOS_PROFILE_DEFAULT};

    #[test]
    fn test_instantiate_subscriber() -> Result<(), RclrsError> {
        let context =
            Context::new(vec![]).expect("Context instantiation is expected to be a success");
        let node = create_node(&context, "test_new_subscriber")?;
        let _subscriber = Subscription::<std_msgs::msg::String>::new(
            &node,
            "test",
            QOS_PROFILE_DEFAULT,
            move |_: std_msgs::msg::String| {},
        )?;

        Ok(())
    }
}

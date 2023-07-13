use std::ffi::CStr;
use std::ffi::CString;
use std::marker::PhantomData;
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex, MutexGuard};

use rosidl_runtime_rs::{Message, RmwMessage};

use crate::error::{RclReturnCode, ToResult};
use crate::qos::QoSProfile;
use crate::{rcl_bindings::*, RclrsError};

mod callback;
mod message_info;
mod readonly_loaned_message;
pub use callback::*;
pub use message_info::*;
pub use readonly_loaned_message::*;

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_subscription_t {}

/// Internal struct used by subscriptions.
pub struct SubscriptionHandle {
    rcl_subscription_mtx: Mutex<rcl_subscription_t>,
    rcl_node_mtx: Arc<Mutex<rcl_node_t>>,
    pub(crate) in_use_by_wait_set: Arc<AtomicBool>,
}

impl SubscriptionHandle {
    pub(crate) fn lock(&self) -> MutexGuard<rcl_subscription_t> {
        self.rcl_subscription_mtx.lock().unwrap()
    }
}

impl Drop for SubscriptionHandle {
    fn drop(&mut self) {
        let rcl_subscription = self.rcl_subscription_mtx.get_mut().unwrap();
        let rcl_node = &mut *self.rcl_node_mtx.lock().unwrap();
        // SAFETY: No preconditions for this function (besides the arguments being valid).
        unsafe {
            rcl_subscription_fini(rcl_subscription, rcl_node);
        }
    }
}

/// Trait to be implemented by concrete [`Subscription`]s.
pub trait SubscriptionBase: Send + Sync {
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
/// The only available way to instantiate subscriptions is via [`Node::create_subscription()`][3], this
/// is to ensure that [`Node`][4]s can track all the subscriptions that have been created.
///
/// [1]: crate::spin_once
/// [2]: crate::spin
/// [3]: crate::Node::create_subscription
/// [4]: crate::Node
pub struct Subscription<T>
where
    T: Message,
{
    pub(crate) handle: Arc<SubscriptionHandle>,
    /// The callback function that runs when a message was received.
    pub callback: Mutex<AnySubscriptionCallback<T>>,
    message: PhantomData<T>,
}

impl<T> Subscription<T>
where
    T: Message,
{
    /// Creates a new subscription.
    pub(crate) fn new<Args>(
        rcl_node_mtx: Arc<Mutex<rcl_node_t>>,
        topic: &str,
        qos: QoSProfile,
        callback: impl SubscriptionCallback<T, Args>,
    ) -> Result<Self, RclrsError>
    // This uses pub(crate) visibility to avoid instantiating this struct outside
    // [`Node::create_subscription`], see the struct's documentation for the rationale
    where
        T: Message,
    {
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_subscription = unsafe { rcl_get_zero_initialized_subscription() };
        let type_support =
            <T as Message>::RmwMsg::get_type_support() as *const rosidl_message_type_support_t;
        let topic_c_string = CString::new(topic).map_err(|err| RclrsError::StringContainsNul {
            err,
            s: topic.into(),
        })?;

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
                &*rcl_node_mtx.lock().unwrap(),
                type_support,
                topic_c_string.as_ptr(),
                &subscription_options,
            )
            .ok()?;
        }

        let handle = Arc::new(SubscriptionHandle {
            rcl_subscription_mtx: Mutex::new(rcl_subscription),
            rcl_node_mtx,
            in_use_by_wait_set: Arc::new(AtomicBool::new(false)),
        });

        Ok(Self {
            handle,
            callback: Mutex::new(callback.into_callback()),
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
            let raw_topic_pointer = rcl_subscription_get_topic_name(&*self.handle.lock());
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
    pub fn take(&self) -> Result<(T, MessageInfo), RclrsError> {
        let mut rmw_message = <T as Message>::RmwMsg::default();
        let message_info = self.take_inner(&mut rmw_message)?;
        Ok((T::from_rmw_message(rmw_message), message_info))
    }

    /// This is a version of take() that returns a boxed message.
    ///
    /// This can be more efficient for messages containing large arrays.
    pub fn take_boxed(&self) -> Result<(Box<T>, MessageInfo), RclrsError> {
        let mut rmw_message = Box::<<T as Message>::RmwMsg>::default();
        let message_info = self.take_inner(&mut *rmw_message)?;
        // TODO: This will still use the stack in general. Change signature of
        // from_rmw_message to allow placing the result in a Box directly.
        let message = Box::new(T::from_rmw_message(*rmw_message));
        Ok((message, message_info))
    }

    // Inner function, to be used by both regular and boxed versions.
    fn take_inner(
        &self,
        rmw_message: &mut <T as Message>::RmwMsg,
    ) -> Result<MessageInfo, RclrsError> {
        let mut message_info = unsafe { rmw_get_zero_initialized_message_info() };
        let rcl_subscription = &mut *self.handle.lock();
        unsafe {
            // SAFETY: The first two pointers are valid/initialized, and do not need to be valid
            // beyond the function call.
            // The latter two pointers are explicitly allowed to be NULL.
            rcl_take(
                rcl_subscription,
                rmw_message as *mut <T as Message>::RmwMsg as *mut _,
                &mut message_info,
                std::ptr::null_mut(),
            )
            .ok()?
        };
        Ok(MessageInfo::from_rmw_message_info(&message_info))
    }

    /// Obtains a read-only handle to a message owned by the middleware.
    ///
    /// When there is no new message, this will return a
    /// [`SubscriptionTakeFailed`][1].
    ///
    /// This is the counterpart to [`Publisher::borrow_loaned_message()`][2]. See its documentation
    /// for more information.
    ///
    /// [1]: crate::RclrsError
    /// [2]: crate::Publisher::borrow_loaned_message
    pub fn take_loaned(&self) -> Result<(ReadOnlyLoanedMessage<'_, T>, MessageInfo), RclrsError> {
        let mut msg_ptr = std::ptr::null_mut();
        let mut message_info = unsafe { rmw_get_zero_initialized_message_info() };
        unsafe {
            // SAFETY: The third argument (message_info) and fourth argument (allocation) may be null.
            // The second argument (loaned_message) contains a null ptr as expected.
            rcl_take_loaned_message(
                &*self.handle.lock(),
                &mut msg_ptr,
                &mut message_info,
                std::ptr::null_mut(),
            )
            .ok()?;
        }
        let read_only_loaned_msg = ReadOnlyLoanedMessage {
            msg_ptr: msg_ptr as *const T::RmwMsg,
            subscription: self,
        };
        Ok((
            read_only_loaned_msg,
            MessageInfo::from_rmw_message_info(&message_info),
        ))
    }
}

impl<T> SubscriptionBase for Subscription<T>
where
    T: Message,
{
    fn handle(&self) -> &SubscriptionHandle {
        &self.handle
    }

    fn execute(&self) -> Result<(), RclrsError> {
        // Immediately evaluated closure, to handle SubscriptionTakeFailed
        // outside this match
        match (|| {
            match &mut *self.callback.lock().unwrap() {
                AnySubscriptionCallback::Regular(cb) => {
                    let (msg, _) = self.take()?;
                    cb(msg)
                }
                AnySubscriptionCallback::RegularWithMessageInfo(cb) => {
                    let (msg, msg_info) = self.take()?;
                    cb(msg, msg_info)
                }
                AnySubscriptionCallback::Boxed(cb) => {
                    let (msg, _) = self.take_boxed()?;
                    cb(msg)
                }
                AnySubscriptionCallback::BoxedWithMessageInfo(cb) => {
                    let (msg, msg_info) = self.take_boxed()?;
                    cb(msg, msg_info)
                }
                AnySubscriptionCallback::Loaned(cb) => {
                    let (msg, _) = self.take_loaned()?;
                    cb(msg)
                }
                AnySubscriptionCallback::LoanedWithMessageInfo(cb) => {
                    let (msg, msg_info) = self.take_loaned()?;
                    cb(msg, msg_info)
                }
            }
            Ok(())
        })() {
            Err(RclrsError::RclError {
                code: RclReturnCode::SubscriptionTakeFailed,
                ..
            }) => {
                // Spurious wakeup – this may happen even when a waitset indicated that this
                // subscription was ready, so it shouldn't be an error.
                Ok(())
            }
            other => other,
        }
    }
}

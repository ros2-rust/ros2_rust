use crate::qos::QoSProfile;
use crate::Node;
use ffi;
use std::ffi::CString;
use std::mem::MaybeUninit;
use thiserror::Error;

struct PublisherOptions {
    handle: ffi::rcl_publisher_options_t,
}

impl PublisherOptions {
    fn from_qos(qos_profile: QoSProfile) -> Self {
        // Safety: Returns a struct that does not have to be freed because
        // it lives on the stack. Cannot fail.
        let mut default_publisher = unsafe { ffi::rcl_publisher_get_default_options() };
        default_publisher.qos = qos_profile.into();
        PublisherOptions {
            handle: default_publisher,
        }
    }
}

#[derive(Error, Debug)]
pub enum PublisherError {
    #[error("The given Parameter has already been initialized")]
    AlreadyInitialized,
    #[error("The given Node is invalid")]
    InvalidNode,
    #[error("One of the provided arguments is invalid")]
    InvalidArgument,
    #[error("Allocating memory failed")]
    BadAllocation,
    #[error("The given topic name is invalid")]
    InvalidTopicName,
    #[error("Unspecified error occured")]
    UnspecifiedError,
}

pub struct Publisher<'node, MsgType> {
    handle: ffi::rcl_publisher_t,
    node_handle: &'node Node<'node>,
    _type: std::marker::PhantomData<MsgType>,
}

impl<'node, T> Publisher<'node, T>
where
    T: rclrs_common::traits::MessageDefinition<T>,
{
    pub(crate) fn new(
        node: &'node Node,
        topic: &str,
        qos_profile: QoSProfile,
    ) -> Result<Self, PublisherError> {
        let options = PublisherOptions::from_qos(qos_profile);
        let type_support = T::get_type_support() as *const ffi::rosidl_message_type_support_t;
        let topic_c_string = CString::new(topic).map_err(|_| PublisherError::InvalidTopicName)?;
        let mut publisher = MaybeUninit::zeroed();

        let node_lock = node.node.lock().unwrap();
        // Safety: Node must be valid, which it is because the Publisher
        // cannot outlive a Node. Topic is a C string whose lifetime
        // is longer than the call to this function.
        let return_code = unsafe {
            ffi::rcl_publisher_init(
                publisher.as_mut_ptr(),
                &*node_lock,
                type_support,
                topic_c_string.as_ptr(),
                &options.handle,
            )
        };
        drop(node_lock);
        // Safety: The initialize function has just been called, which means
        // that `publisher` can be assumed to be intialized.
        let publisher = unsafe { publisher.assume_init() };

        match return_code as u32 {
            ffi::RCL_RET_OK => Ok(Publisher::<T> {
                handle: publisher,
                node_handle: node,
                _type: std::marker::PhantomData::<T>,
            }),
            ffi::RCL_RET_TOPIC_NAME_INVALID => Err(PublisherError::InvalidTopicName),
            ffi::RCL_RET_NODE_INVALID => Err(PublisherError::InvalidNode),
            ffi::RCL_RET_ALREADY_INIT => Err(PublisherError::AlreadyInitialized),
            ffi::RCL_RET_INVALID_ARGUMENT => Err(PublisherError::InvalidArgument),
            ffi::RCL_RET_BAD_ALLOC => Err(PublisherError::BadAllocation),
            _ => Err(PublisherError::UnspecifiedError),
        }
    }
}

impl<T> Drop for Publisher<'_, T> {
    fn drop(&mut self) {
        let mut node_lock = self.node_handle.node.lock().unwrap();
        // Safety: This function can have four different errors,
        // - Invalid argument
        //      > Occurs when one of the two parameters is null or not the
        //      > correct type. We can guarantee that this cannot happen.
        // - Publisher invalid
        //      > Occurs when the publisher has not been properly initialized
        //      > or when the publisher has already been dropped. If we assume
        //      > that a double-free cannot happen and that the constructor of
        //      > Publisher is safe, then we can guarantee that this will not
        //      > happen.
        // - Node invalid
        //      > The lifetime of the Publisher is at most as long as the
        //      > lifetime of the Node. Therefore, a Publisher cannot outlive
        //      > a Node and Nodes are always valid during their lifetime.
        //      > Therefore, we can guarantee that this will not happen.
        // - Unknown error
        //      > Could still happen for unknown reasons, we should panic
        //      > because it is not possible to make any assumptions on the
        //      > state of the system if this error is returned.
        let return_code = unsafe { ffi::rcl_publisher_fini(&mut self.handle, &mut *node_lock) };
        if return_code as u32 != ffi::RCL_RET_OK {
            panic!("Unspecified error occured while dropping Publisher")
        }
    }
}
impl<'node> Node<'node> {
    /// Create a [`Publisher`](crate::Publisher) on a topic for a specified message.
    /// # Arguments
    /// * `topic` - The topic that messages will be published on. Topic names
    ///             must comply [these rules](https://design.ros2.org/articles/topic_and_service_names.html#ros-2-topic-and-service-name-constraints).
    /// * `qos_profile` - [`Quality of Service profile`](crate::QoSProfile) that this publisher will
    ///             use when it is publishing messages.
    /// ```
    pub fn create_publisher<MsgType>(
        &'node self,
        topic: &str,
        qos_profile: QoSProfile,
    ) -> Result<Publisher<MsgType>, PublisherError>
    where
        MsgType: rclrs_common::traits::MessageDefinition<MsgType>,
    {
        Publisher::new(self, topic, qos_profile)
    }
}

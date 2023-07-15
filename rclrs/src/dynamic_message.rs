//! Functionality for working with messages whose type is not statically known.
//!
//! This is useful for writing generic tools such as introspection tools, bridges to
//! other communication systems, or nodes that manipulate messages à la `topic_tools`.
//!
//! The central type of this module is [`DynamicMessage`].

use std::fmt::{self, Display};
use std::path::PathBuf;
use std::sync::Arc;

use crate::rcl_bindings::rosidl_typesupport_introspection_c__MessageMembers_s as rosidl_message_members_t;
use crate::rcl_bindings::*;

mod error;
pub use error::*;

/// Factory for constructing messages in a certain package dynamically.
///
/// This is the result of loading the introspection type support library (which is a per-package
/// operation), whereas [`DynamicMessageMetadata`] is the result of loading the data related to
/// the message from the library.
//
// Theoretically it could be beneficial to make this struct public so users can "cache"
// the library loading, but unless a compelling use case comes up, I don't think it's
// worth the complexity.
//
// Under the hood, this is an `Arc<libloading::Library>`, so if this struct and the
// [`DynamicMessageMetadata`] and [`DynamicMessage`] structs created from it are dropped,
// the library will be unloaded. This shared ownership ensures that the type_support_ptr
// is always valid.
struct DynamicMessagePackage {
    introspection_type_support_library: Arc<libloading::Library>,
    package: String,
}

/// A parsed/validated message type name of the form `<package_name>/msg/<type_name>`.
#[derive(Clone, Debug, PartialEq, Eq)]
struct MessageTypeName {
    /// The package name, which acts as a namespace.
    pub package_name: String,
    /// The name of the message type in the package.
    pub type_name: String,
}

/// A runtime representation of the message "class".
///
/// This is not an instance of a message itself, but it
/// can be used as a factory to create message instances.
#[derive(Clone)]
pub struct DynamicMessageMetadata {
    #[allow(dead_code)]
    message_type: MessageTypeName,
    // The library needs to be kept loaded in order to keep the type_support_ptr valid.
    #[allow(dead_code)]
    introspection_type_support_library: Arc<libloading::Library>,
    #[allow(dead_code)]
    type_support_ptr: *const rosidl_message_type_support_t,
    #[allow(dead_code)]
    fini_function: unsafe extern "C" fn(*mut std::os::raw::c_void),
}

// ========================= impl for DynamicMessagePackage =========================

/// This is an analogue of rclcpp::get_typesupport_library.
fn get_type_support_library(
    package_name: &str,
    type_support_identifier: &str,
) -> Result<Arc<libloading::Library>, DynamicMessageError> {
    use DynamicMessageError::RequiredPrefixNotSourced;
    // Creating this is pretty cheap, it just parses an env var
    let ament = ament_rs::Ament::new().map_err(|_| RequiredPrefixNotSourced {
        package: package_name.to_owned(),
    })?;
    let prefix = PathBuf::from(ament.find_package(package_name).ok_or(
        RequiredPrefixNotSourced {
            package: package_name.to_owned(),
        },
    )?);
    #[cfg(target_os = "windows")]
    let library_path = prefix.join("bin").join(format!(
        "{}__{}.dll",
        &package_name, type_support_identifier
    ));
    #[cfg(target_os = "macos")]
    let library_path = prefix.join("lib").join(format!(
        "lib{}__{}.dylib",
        &package_name, type_support_identifier
    ));
    #[cfg(all(not(target_os = "windows"), not(target_os = "macos")))]
    let library_path = prefix.join("lib").join(format!(
        "lib{}__{}.so",
        &package_name, type_support_identifier
    ));
    Ok({
        // SAFETY: This function is unsafe because it may execute initialization/termination routines
        // contained in the library. A type support library should not cause problems there.
        let lib = unsafe { libloading::Library::new(library_path) };
        let lib = lib.map_err(DynamicMessageError::LibraryLoadingError)?;
        Arc::new(lib)
    })
}

/// This is an analogue of rclcpp::get_typesupport_handle.
///
/// It is unsafe because it would be theoretically possible to pass in a library that has
/// the expected symbol defined, but with an unexpected type.
unsafe fn get_type_support_handle(
    type_support_library: &libloading::Library,
    type_support_identifier: &str,
    message_type: &MessageTypeName,
) -> Result<*const rosidl_message_type_support_t, DynamicMessageError> {
    let symbol_name = format!(
        "{}__get_message_type_support_handle__{}__msg__{}",
        type_support_identifier, &message_type.package_name, &message_type.type_name
    );

    // SAFETY: We know that the symbol has this type, from the safety requirement of this function.
    let getter: libloading::Symbol<unsafe extern "C" fn() -> *const rosidl_message_type_support_t> = /* unsafe */ {
        type_support_library
            .get(symbol_name.as_bytes())
            .map_err(|_| DynamicMessageError::InvalidMessageType)?
    };

    // SAFETY: The caller is responsible for keeping the library loaded while
    // using this pointer.
    let type_support_ptr = /* unsafe */ { getter() };
    Ok(type_support_ptr)
}

const INTROSPECTION_TYPE_SUPPORT_IDENTIFIER: &str = "rosidl_typesupport_introspection_c";

impl DynamicMessagePackage {
    /// Creates a new `DynamicMessagePackage`.
    ///
    /// This dynamically loads a type support library for the specified package.
    pub fn new(package_name: impl Into<String>) -> Result<Self, DynamicMessageError> {
        let package_name = package_name.into();
        Ok(Self {
            introspection_type_support_library: get_type_support_library(
                &package_name,
                INTROSPECTION_TYPE_SUPPORT_IDENTIFIER,
            )?,
            package: package_name,
        })
    }

    pub(crate) fn message_metadata(
        &self,
        type_name: impl Into<String>,
    ) -> Result<DynamicMessageMetadata, DynamicMessageError> {
        let message_type = MessageTypeName {
            package_name: self.package.clone(),
            type_name: type_name.into(),
        };
        // SAFETY: The symbol type of the type support getter function can be trusted
        // assuming the install dir hasn't been tampered with.
        // The pointer returned by this function is kept valid by keeping the library loaded.
        let type_support_ptr = unsafe {
            get_type_support_handle(
                self.introspection_type_support_library.as_ref(),
                INTROSPECTION_TYPE_SUPPORT_IDENTIFIER,
                &message_type,
            )?
        };
        // SAFETY: The pointer returned by get_type_support_handle() is always valid.
        let type_support = unsafe { &*type_support_ptr };
        debug_assert!(!type_support.data.is_null());
        let message_members: &rosidl_message_members_t =
            // SAFETY: The data pointer is supposed to be always valid.
            unsafe { &*(type_support.data as *const rosidl_message_members_t) };
        // The fini function will always exist.
        let fini_function = message_members.fini_function.unwrap();
        let metadata = DynamicMessageMetadata {
            message_type,
            introspection_type_support_library: Arc::clone(
                &self.introspection_type_support_library,
            ),
            type_support_ptr,
            fini_function,
        };
        Ok(metadata)
    }
}

// ========================= impl for MessageTypeName =========================

impl TryFrom<&str> for MessageTypeName {
    type Error = DynamicMessageError;
    fn try_from(full_message_type: &str) -> Result<Self, Self::Error> {
        let mut parts = full_message_type.split('/');
        use DynamicMessageError::InvalidMessageTypeSyntax;
        let package_name = parts
            .next()
            .ok_or(InvalidMessageTypeSyntax {
                input: full_message_type.to_owned(),
            })?
            .to_owned();
        if Some("msg") != parts.next() {
            return Err(InvalidMessageTypeSyntax {
                input: full_message_type.to_owned(),
            });
        };
        let type_name = parts
            .next()
            .ok_or(InvalidMessageTypeSyntax {
                input: full_message_type.to_owned(),
            })?
            .to_owned();
        if parts.next().is_some() {
            return Err(InvalidMessageTypeSyntax {
                input: full_message_type.to_owned(),
            });
        }
        Ok(Self {
            package_name,
            type_name,
        })
    }
}

impl Display for MessageTypeName {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}/msg/{}", &self.package_name, &self.type_name)
    }
}

// ========================= impl for DynamicMessageMetadata =========================

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for DynamicMessageMetadata {}

// SAFETY: The type_support_ptr member is the one that makes this type not implement Sync
// automatically, but it is not used for interior mutability.
unsafe impl Sync for DynamicMessageMetadata {}

impl DynamicMessageMetadata {
    /// Loads the metadata for the given message type.
    ///
    /// See [`DynamicMessage::new()`] for the expected format of the `full_message_type`.
    pub fn new(full_message_type: &str) -> Result<Self, DynamicMessageError> {
        let MessageTypeName {
            package_name,
            type_name,
        } = full_message_type.try_into()?;
        let pkg = DynamicMessagePackage::new(package_name)?;
        pkg.message_metadata(type_name)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_send<T: Send>() {}
    fn assert_sync<T: Sync>() {}

    #[test]
    fn all_types_are_sync_and_send() {
        assert_send::<DynamicMessageMetadata>();
        assert_sync::<DynamicMessageMetadata>();
    }

    #[test]
    fn invalid_message_type_name() {
        assert!(matches!(
            DynamicMessageMetadata::new("x"),
            Err(DynamicMessageError::InvalidMessageTypeSyntax { .. })
        ));
        assert!(matches!(
            DynamicMessageMetadata::new("x/y"),
            Err(DynamicMessageError::InvalidMessageTypeSyntax { .. })
        ));
        assert!(matches!(
            DynamicMessageMetadata::new("x//y"),
            Err(DynamicMessageError::InvalidMessageTypeSyntax { .. })
        ));
        assert!(matches!(
            DynamicMessageMetadata::new("x/msg/y"),
            Err(DynamicMessageError::RequiredPrefixNotSourced { .. })
        ));
        assert!(matches!(
            DynamicMessageMetadata::new("x/msg/y/z"),
            Err(DynamicMessageError::InvalidMessageTypeSyntax { .. })
        ));
    }
}

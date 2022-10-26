use std::error::Error;
use std::fmt;

/// An error related to creating a dynamic message based on the name of the message's type.
#[derive(Debug)]
pub enum DynamicMessageError {
    /// The type support library was not found because no matching prefix was sourced.
    RequiredPrefixNotSourced {
        /// The package that was not found.
        package: String,
    },
    /// The message type does not have the shape `<package>/msg/<msg_name>`.
    InvalidMessageTypeSyntax {
        /// The message type passed to rclrs.
        input: String,
    },
    /// The message type could not be found in the package.
    InvalidMessageType,
    /// The operation expected a dynamic message of a different type.
    MessageTypeMismatch,
    /// Loading the type support library failed.
    LibraryLoadingError(libloading::Error),
}

impl fmt::Display for DynamicMessageError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::RequiredPrefixNotSourced { package } => {
                write!(f, "Package '{}' was not found in any prefix", package)
            }
            Self::InvalidMessageTypeSyntax { input } => write!(
                f,
                "The message type '{}' does not have the form <package>/msg/<msg_name>",
                input
            ),
            Self::InvalidMessageType => write!(f, "The message type was not found in the package"),
            Self::MessageTypeMismatch => write!(
                f,
                "The operation expected a dynamic message of a different type"
            ),
            Self::LibraryLoadingError(_) => write!(f, "Loading the type support library failed"),
        }
    }
}

impl Error for DynamicMessageError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            DynamicMessageError::LibraryLoadingError(lle) => Some(lle).map(|e| e as &dyn Error),
            _ => None,
        }
    }
}

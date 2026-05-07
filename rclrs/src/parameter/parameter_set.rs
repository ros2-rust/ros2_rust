use crate::{DeclarationError, NodeState};

/// A collection of parameters that can be declared together on a node.
///
/// This trait is typically derived using `#[derive(ParameterSet)]` from the
/// `rclrs` crate. The derive macro generates builder calls for each field
/// based on struct annotations.
///
/// # Example
///
/// ```no_run
/// use rclrs::*;
///
/// #[derive(ParameterSet)]
/// struct MyParams {
///     #[param(default = 50.0, description = "Max speed")]
///     speed: MandatoryParameter<f64>,
/// }
///
/// let executor = Context::default_from_env().unwrap().create_basic_executor();
/// let node = executor.create_node("my_node").unwrap();
/// let params: MyParams = node.declare_parameter_set().unwrap();
/// ```
pub trait ParameterSet: Sized {
    /// The default namespace for this parameter set.
    ///
    /// Derived from the struct name converted to snake_case.
    /// Override with `#[parameters(namespace = "custom")]`.
    /// Returns `""` when `#[parameters(flatten)]` is used.
    ///
    /// Note: `flatten` only affects this struct's own namespace. It does not
    /// propagate to nested `ParameterSet` fields — those still use their own
    /// `default_namespace()` unless individually marked with `#[param(flatten)]`.
    fn default_namespace() -> &'static str;

    /// Declares all parameters on the node under the given prefix.
    ///
    /// An empty prefix means parameters are declared at root level.
    /// A non-empty prefix results in parameter names like `"{prefix}.{field}"`.
    fn declare(node: &NodeState, prefix: &str) -> Result<Self, DeclarationError>;
}

/// Helper utilities used by the `ParameterSet` derive macro.
///
/// These are not part of the public API and may change without notice.
#[doc(hidden)]
pub mod __private {
    /// Builds a parameter name from a prefix and field name.
    ///
    /// - Empty prefix: returns the field name directly.
    /// - Non-empty prefix: returns `"{prefix}.{field}"`.
    pub fn param_name(prefix: &str, field: &str) -> String {
        if prefix.is_empty() {
            field.to_string()
        } else {
            format!("{prefix}.{field}")
        }
    }
}

#[cfg(test)]
#[path = "parameter_set_tests.rs"]
mod tests;

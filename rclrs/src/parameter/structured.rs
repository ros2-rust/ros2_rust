//! This module provides the trait [`StructuredParameters`] default implementations for declaring parameters in structured fashion.
//! [`rclrs_proc_macros::StructuredParameters`] provides a macro to derive the trait for structs automatically.
//!

use crate::NodeState;

/// Marker trait for implementing [`StructuredParameters`] where a default value cannot be specified.
/// This is usually the case for container types, that are not represented by any actual parameter.
pub enum DefaultForbidden {}

/// Types implementing this trait can delcare their parameters with[`NodeState::declare_parameters`].
/// The trait can be automatically derived using [`rclrs_proc_macros`] if:
/// - if the type is a struct
/// - all attributes implement [`StructuredParameters`]
pub trait StructuredParameters: Sized {
    /// Declares all parameters in ros node.
    ///
    /// # Parameters
    ///
    /// - `node`: The ros node to declare parameters for.
    /// - `name`: The name of the parameter. Nested parameters are recursively declared with "{name}.{field_name}" if the name is not empty else "{field_name}".
    /// - `default`: The default value for the paramter.
    /// - `description` The description of the parameter
    ///
    /// # Returns
    ///
    /// [`Result`] containing the declared structured parameters or [`crate::DeclarationError`]
    fn declare_structured<T>(
        node: &NodeState,
        name: &str,
        default: Option<T>,
        description: impl Into<std::sync::Arc<str>>,
        constraints: impl Into<std::sync::Arc<str>>,
        ignore_override: bool,
        discard_mismatching_prior_value: bool,
    ) -> core::result::Result<Self, crate::DeclarationError>
    where
        Self: StructuredParametersMeta<T>,
    {
        Self::declare_structured_(
            node,
            name,
            default,
            description,
            constraints,
            ignore_override,
            discard_mismatching_prior_value,
        )
    }
}

/// Helper trait to unify the default value type with generic container types like
/// - [`crate::MandatoryParameter<T>`]
/// - [`crate::OptionalParameter<T>`]
/// - [`crate::ReadOnlyParameter<T>`]
///
/// In these cases the [`Self`] != [`T`] and forces the usage of a generic trait.
/// However, a generic trait also requires annotating this default value in the derive macro.
/// For the container based structured parameters [`T`] is always [`DefaultForbidden`]
/// and therefore we can hide this form the trait + macro by using this helper trait.
/// The previously mentioned leaf types that actually hold parameters, are to be implemented manually anyway.
///
pub trait StructuredParametersMeta<T>: Sized {
    /// See [`StructuredParameters::declare_structured`]
    fn declare_structured_(
        node: &NodeState,
        name: &str,
        default: Option<T>,
        description: impl Into<std::sync::Arc<str>>,
        constraints: impl Into<std::sync::Arc<str>>,
        ignore_override: bool,
        discard_mismatching_prior_value: bool,
    ) -> core::result::Result<Self, crate::DeclarationError>;
}

impl NodeState {
    /// Declares all nested parameters of the [`StructuredParameters`].
    /// Parameter naming recursively follows "`name`.`field_name`" or "`field_name`" if the initial `name` is empty.
    pub fn declare_parameters<T, T0>(
        &self,
        name: &str,
    ) -> core::result::Result<T, crate::DeclarationError>
    where
        T: StructuredParameters + StructuredParametersMeta<T0>,
    {
        T::declare_structured(self, name, None, "", "", false, false)
    }
}

impl<T: crate::ParameterVariant> StructuredParameters for crate::MandatoryParameter<T> {}
impl<T: crate::ParameterVariant> StructuredParametersMeta<T> for crate::MandatoryParameter<T> {
    fn declare_structured_(
        node: &NodeState,
        name: &str,
        default: Option<T>,
        description: impl Into<std::sync::Arc<str>>,
        constraints: impl Into<std::sync::Arc<str>>,
        ignore_override: bool,
        discard_mismatching_prior_value: bool,
    ) -> core::result::Result<Self, crate::DeclarationError> {
        let builder = node.declare_parameter(name);
        let builder = match default {
            Some(default) => builder.default(default),
            None => builder,
        };
        let builder = match ignore_override {
            true => builder.ignore_override(),
            false => builder,
        };
        let builder = match discard_mismatching_prior_value {
            true => builder.discard_mismatching_prior_value(),
            false => builder,
        };
        builder
            .description(description)
            .constraints(constraints)
            .mandatory()
    }
}
impl<T: crate::ParameterVariant> StructuredParameters for crate::ReadOnlyParameter<T> {}
impl<T: crate::ParameterVariant> StructuredParametersMeta<T> for crate::ReadOnlyParameter<T> {
    fn declare_structured_(
        node: &NodeState,
        name: &str,
        default: Option<T>,
        description: impl Into<std::sync::Arc<str>>,
        constraints: impl Into<std::sync::Arc<str>>,
        ignore_override: bool,
        discard_mismatching_prior_value: bool,
    ) -> core::result::Result<Self, crate::DeclarationError> {
        let builder = node.declare_parameter(name);
        let builder = match default {
            Some(default) => builder.default(default),
            None => builder,
        };
        let builder = match ignore_override {
            true => builder.ignore_override(),
            false => builder,
        };
        let builder = match discard_mismatching_prior_value {
            true => builder.discard_mismatching_prior_value(),
            false => builder,
        };
        builder
            .description(description)
            .constraints(constraints)
            .read_only()
    }
}
impl<T: crate::ParameterVariant> StructuredParameters for crate::OptionalParameter<T> {}
impl<T: crate::ParameterVariant> StructuredParametersMeta<T> for crate::OptionalParameter<T> {
    fn declare_structured_(
        node: &NodeState,
        name: &str,
        default: Option<T>,
        description: impl Into<std::sync::Arc<str>>,
        constraints: impl Into<std::sync::Arc<str>>,
        ignore_override: bool,
        discard_mismatching_prior_value: bool,
    ) -> core::result::Result<Self, crate::DeclarationError> {
        let builder = node.declare_parameter(name);
        let builder = match default {
            Some(default) => builder.default(default),
            None => builder,
        };
        let builder = match ignore_override {
            true => builder.ignore_override(),
            false => builder,
        };
        let builder = match discard_mismatching_prior_value {
            true => builder.discard_mismatching_prior_value(),
            false => builder,
        };

        //builder.discriminate(f)
        //builder.range(range)

        builder
            .description(description)
            .constraints(constraints)
            .optional()
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use crate as rclrs;
    use rclrs::{parameter::structured::*, CreateBasicExecutor};
    use rclrs_macros::StructuredParameters;

    #[derive(StructuredParameters, Debug)]
    struct SimpleStructuredParameters {
        _mandatory: rclrs::MandatoryParameter<f64>,
        _optional: rclrs::OptionalParameter<f64>,
        _readonly: rclrs::ReadOnlyParameter<f64>,
    }

    #[test]
    fn test_simple_structured_parameters() {
        let args: Vec<String> = [
            "test",
            "--ros-args",
            "-p",
            "_mandatory:=1.0",
            "-p",
            "_optional:=1.0",
            "-p",
            "_readonly:=1.0",
        ]
        .into_iter()
        .map(str::to_string)
        .collect();

        let context = crate::Context::new(args, rclrs::InitOptions::default()).unwrap();
        let exec = context.create_basic_executor();
        let node = exec.create_node(rclrs::NodeOptions::new("test")).unwrap();
        let _params: SimpleStructuredParameters = node.declare_parameters("").unwrap();
        println!("{:?}", _params);
    }

    #[derive(StructuredParameters, Debug)]
    struct NestedStructuredParameters {
        _simple: SimpleStructuredParameters,
        _mandatory: rclrs::MandatoryParameter<Arc<str>>,
    }

    #[test]
    fn test_nested_structured_parameters() {
        let args: Vec<String> = [
            "test",
            "--ros-args",
            "-p",
            "nested._simple._mandatory:=1.0",
            "-p",
            "nested._simple._optional:=1.0",
            "-p",
            "nested._simple._readonly:=1.0",
            "-p",
            "nested._mandatory:=foo",
        ]
        .into_iter()
        .map(str::to_string)
        .collect();

        let context = crate::Context::new(args, rclrs::InitOptions::default()).unwrap();
        let exec = context.create_basic_executor();
        let node = exec.create_node(rclrs::NodeOptions::new("test")).unwrap();

        let _params: NestedStructuredParameters = node.declare_parameters("nested").unwrap();
        println!("{:?}", _params);
    }

    #[derive(Debug, StructuredParameters)]
    struct SimpleStructuredParametersWithDefaults {
        #[param(default = 42.0)]
        _mandatory: rclrs::MandatoryParameter<f64>,
        #[param(default = 42.0)]
        _optional: rclrs::OptionalParameter<f64>,
        #[param(default = Arc::from("test"))]
        _readonly: rclrs::ReadOnlyParameter<Arc<str>>,
    }

    #[test]
    fn test_simple_structured_parameters_with_defaults() {
        let args: Vec<String> = ["test", "--ros-args"]
            .into_iter()
            .map(str::to_string)
            .collect();
        let context = crate::Context::new(args, rclrs::InitOptions::default()).unwrap();
        let exec = context.create_basic_executor();
        let node = exec.create_node(rclrs::NodeOptions::new("test")).unwrap();
        let _params: SimpleStructuredParametersWithDefaults = node.declare_parameters("").unwrap();
        println!("{:?}", _params);
    }
    #[derive(Debug, StructuredParameters)]
    struct SimpleStructuredParametersWithDefaultsAndDescriptions {
        #[param(default = 42.0, ignore_override, description = "_mandatory")]
        _mandatory: rclrs::MandatoryParameter<f64>,
        #[param(default = 42.0, description = "_optional")]
        _optional: rclrs::OptionalParameter<f64>,
        #[param(default = Arc::from("test"), description = "_readonly")]
        _readonly: rclrs::ReadOnlyParameter<Arc<str>>,
    }

    #[test]
    fn test_simple_structured_parameters_with_defaults_and_descriptions() {
        let args: Vec<String> = ["test", "--ros-args"]
            .into_iter()
            .map(str::to_string)
            .collect();
        let context = crate::Context::new(args, rclrs::InitOptions::default()).unwrap();
        let exec = context.create_basic_executor();
        let node = exec.create_node(rclrs::NodeOptions::new("test")).unwrap();
        let _params: SimpleStructuredParametersWithDefaultsAndDescriptions =
            node.declare_parameters("").unwrap();
        println!("{:?}", _params);
    }
}

//! Declaring a whole tree of parameters from a Rust struct.
//!
//! A [`ParameterSet`] is a struct of plain Rust values whose fields correspond to ROS 2
//! parameters. Implement it with `#[derive(ParameterSet)]` and declare it with
//! [`NodeState::declare_parameters`], [`NodeState::retain_parameters`] or
//! [`NodeState::load_parameters`].
//!
//! Each field of the struct is declared through [`DeclareField`], which is what decides whether
//! the field is a single parameter, a nested set of parameters, or something else entirely. The
//! derive macro emits the same code for every field and lets the type system pick the
//! implementation, so a nested set needs no annotation to be recognised as one. Any type with a
//! `DeclareField` implementation can be used as a field, including types defined outside rclrs.
//!
//! [`NodeState::declare_parameters`]: crate::NodeState::declare_parameters
//! [`NodeState::retain_parameters`]: crate::NodeState::retain_parameters
//! [`NodeState::load_parameters`]: crate::NodeState::load_parameters

use std::{fmt::Debug, path::PathBuf, sync::Arc};

use crate::{
    AvailableValues, DeclarationError, NodeState, ParameterBuilder, ParameterValue,
    ParameterVariant,
};

/// Joins a parameter namespace prefix with a name, with the `.` separator ROS 2 uses for nested
/// parameters. Either part may be empty, in which case the other is used on its own.
///
/// Useful when implementing [`DeclareField`] for a type that declares more than one parameter.
///
/// ```
/// # use rclrs::join_parameter_name;
/// assert_eq!(join_parameter_name("limits", "max_force"), "limits.max_force");
/// assert_eq!(join_parameter_name("", "max_force"), "max_force");
/// assert_eq!(join_parameter_name("arm", ""), "arm");
/// ```
pub fn join_parameter_name(prefix: &str, name: &str) -> String {
    match (prefix.is_empty(), name.is_empty()) {
        (true, _) => name.to_string(),
        (false, true) => prefix.to_string(),
        (false, false) => format!("{prefix}.{name}"),
    }
}

/// A parameter declaration in a set failed, naming the parameter responsible.
#[derive(Debug, PartialEq, Eq)]
pub struct ParameterSetError {
    /// Fully qualified name of the parameter that could not be declared, including the
    /// namespaces of any sets it is nested in.
    pub name: String,
    /// Why that parameter could not be declared.
    pub source: DeclarationError,
}

impl ParameterSetError {
    /// Attaches a parameter name to a declaration failure.
    pub fn new(name: impl Into<String>, source: DeclarationError) -> Self {
        Self {
            name: name.into(),
            source,
        }
    }
}

impl std::fmt::Display for ParameterSetError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "failed to declare parameter '{}': {}",
            self.name, self.source
        )
    }
}

impl std::error::Error for ParameterSetError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(&self.source)
    }
}

/// A struct whose fields describe a group of ROS 2 parameters.
///
/// Implemented by `#[derive(ParameterSet)]`. The struct holds plain values, and the corresponding
/// live parameter handles are [`Self::Handles`], which the derive macro generates alongside it.
///
/// # Example
///
/// ```no_run
/// use rclrs::*;
///
/// /// Configuration for a differential drive controller.
/// #[derive(ParameterSet, Debug)]
/// struct DriveConfig {
///     /// Maximum forward speed in m/s.
///     #[param(default = 1.5, range = 0.0..=10.0)]
///     max_speed: f64,
///     /// Names of the wheel joints.
///     #[param(default = ["left_wheel", "right_wheel"])]
///     wheels: Vec<String>,
///     /// Safety limits. Nested sets need no annotation.
///     limits: Limits,
/// }
///
/// #[derive(ParameterSet, Debug)]
/// struct Limits {
///     /// Maximum motor force in N.
///     #[param(default = 100.0)]
///     max_force: f64,
/// }
///
/// let executor = Context::default_from_env()?.create_basic_executor();
/// let node = executor.create_node("drive_controller")?;
///
/// // Declares max_speed, wheels and limits.max_force.
/// let config: DriveConfig = node.load_parameters()?;
/// println!("{config:?}");
/// # Ok::<(), RclrsError>(())
/// ```
pub trait ParameterSet: Sized {
    /// The live handles for the parameters of this set, one field per parameter.
    type Handles: ParameterSetHandles<Values = Self>;

    /// The namespace this set declares its parameters under when it is declared at the top
    /// level. `""` places them at the node's root, which is the default and makes the struct
    /// mirror the shape of a parameter YAML file. Set it with
    /// `#[parameters(namespace = "...")]`.
    const NAMESPACE: &'static str;

    /// Declares every parameter of this set under `prefix`.
    ///
    /// `default`, if given, supplies the default value of each field that does not specify one
    /// of its own with `#[param(default = ...)]`.
    fn declare(
        node: &NodeState,
        prefix: &str,
        default: Option<Self>,
    ) -> Result<Self::Handles, ParameterSetError>;
}

/// The live parameter handles of a [`ParameterSet`], generated by `#[derive(ParameterSet)]`.
pub trait ParameterSetHandles {
    /// The struct of plain values these handles were declared from.
    type Values;

    /// Reads every parameter in the set and returns them as plain values.
    ///
    /// The parameters are read one after another, so a snapshot taken during a concurrent update
    /// may mix old and new values. Read individual handles if two parameters must agree.
    fn snapshot(&self) -> Self::Values;
}

/// Options for declaring one field of a [`ParameterSet`], collected from its `#[param(...)]`
/// attributes.
///
/// `V` is the field's [`DeclareField::Value`] and `R` its [`DeclareField::Range`]. Build one
/// with struct update syntax:
///
/// ```
/// # use rclrs::{FieldSpec, ParameterRange};
/// let spec: FieldSpec<f64, ParameterRange<f64>> = FieldSpec {
///     default: Some(1.5),
///     description: "Maximum forward speed in m/s.",
///     ..Default::default()
/// };
/// ```
pub struct FieldSpec<V, R> {
    /// Value to use when no override and no prior value is available.
    pub default: Option<V>,
    /// Human readable description for the parameter descriptor.
    pub description: &'static str,
    /// Human readable constraints for the parameter descriptor. Empty means the field's type
    /// decides, via [`ParameterVariant::type_constraints`].
    pub constraints: &'static str,
    /// Valid range of values.
    pub range: R,
    /// Whether to ignore an override provided for this parameter.
    pub ignore_override: bool,
    /// Whether to discard, rather than reject, a prior value of a mismatching type.
    pub discard_mismatching_prior_value: bool,
    /// Rejects values before they are applied, including values arriving over the parameter
    /// services.
    pub validate: Option<ValidateFn<V>>,
    /// Chooses the initial value from those available.
    pub discriminate: Option<DiscriminateFn<V>>,
}

/// The type of a [`FieldSpec::validate`] callback.
pub type ValidateFn<V> = Box<dyn Fn(&V) -> Result<(), String> + Send + Sync>;

/// The type of a [`FieldSpec::discriminate`] callback.
pub type DiscriminateFn<V> = Box<dyn for<'a> FnOnce(AvailableValues<'a, V>) -> Option<V>>;

impl<V, R: Default> Default for FieldSpec<V, R> {
    fn default() -> Self {
        Self {
            default: None,
            description: "",
            constraints: "",
            range: R::default(),
            ignore_override: false,
            discard_mismatching_prior_value: false,
            validate: None,
            discriminate: None,
        }
    }
}

impl<V, R: Debug> Debug for FieldSpec<V, R> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("FieldSpec")
            .field("default", &self.default.as_ref().map(|_| ".."))
            .field("description", &self.description)
            .field("constraints", &self.constraints)
            .field("range", &self.range)
            .field("ignore_override", &self.ignore_override)
            .field(
                "discard_mismatching_prior_value",
                &self.discard_mismatching_prior_value,
            )
            .field("validate", &self.validate.as_ref().map(|_| ".."))
            .field("discriminate", &self.discriminate.as_ref().map(|_| ".."))
            .finish()
    }
}

/// Marks a [`ParameterSet`] field that is declared as a writable parameter.
///
/// Whether the parameter is mandatory or optional follows from the field's type: `T` is
/// mandatory, `Option<T>` is optional.
#[derive(Clone, Copy, Debug, Default)]
pub struct Writable;

/// Marks a [`ParameterSet`] field that is declared as a read-only parameter, from
/// `#[param(read_only)]`.
#[derive(Clone, Copy, Debug, Default)]
pub struct ReadOnly;

/// A type that can be a field of a [`ParameterSet`].
///
/// This is the extension point of the parameter set machinery. `#[derive(ParameterSet)]` emits
/// the same code for every field and calls this trait to declare it. What a field *is*, whether a
/// single parameter, a nested set or a map of sets, is decided entirely by which implementation
/// applies to its type. Implement it for your own types to use them as parameters.
///
/// `M` selects the mode the field was declared in: [`Writable`] or [`ReadOnly`]. A type that
/// cannot sensibly be read-only simply does not implement `DeclareField<ReadOnly>`.
#[diagnostic::on_unimplemented(
    message = "`{Self}` cannot be used as a ROS 2 parameter",
    label = "not a parameter type",
    note = "if this is a group of parameters, add `#[derive(ParameterSet)]` to it",
    note = "if this is a single value, implement `ParameterVariant` for it and then call \
            `declare_parameter_field!` on it",
    note = "built-in parameter types are bool, i64, f64, f32, i8, i16, i32, u8, u16, u32, \
            String, PathBuf, ParameterValue, a Vec of any of those, the Arc<[..]> forms of the \
            ROS 2 array types, and `Option<T>` of any of them",
    note = "u64, usize, i128 and u128 are not parameter types because a parameter value is \
            stored as an i64, and every way of handling a value outside its range would \
            silently change it. Use i64, or u32 if the value must be unsigned"
)]
pub trait DeclareField<M = Writable>: Sized {
    /// The value type that `default`, `validate` and `discriminate` operate on. This is `T` for
    /// both `T` and `Option<T>`, since an optional parameter's value is still a `T` when set.
    type Value;

    /// The live handle for this field, stored in the set's handles struct.
    type Handle;

    /// The range type accepted by `#[param(range = ...)]`, in the field's own units. `()` for
    /// fields that cannot have a range.
    type Range: Default;

    /// Declares this field as the parameter (or parameters) called `name`.
    fn declare(
        node: &NodeState,
        name: &str,
        spec: FieldSpec<Self::Value, Self::Range>,
    ) -> Result<Self::Handle, ParameterSetError>;

    /// Reads the field's current value back out of its handle.
    fn snapshot(handle: &Self::Handle) -> Self;

    /// Converts a value of the field's own type into the default to declare with.
    ///
    /// This is the identity for `Option<T>` fields, where an absent value means "no default",
    /// and wraps in `Some` for everything else. It is what lets a set's default value be
    /// destructured field by field without the caller knowing which fields are optional.
    fn into_default(self) -> Option<Self::Value>;
}

/// A [`ParameterSet`] field that can be flattened into its parent's namespace with
/// `#[param(flatten)]`.
///
/// Only sets implement this: flattening an individual parameter would give it its parent's name,
/// which is never what is meant.
#[diagnostic::on_unimplemented(
    message = "`{Self}` cannot be flattened into the parameter set that contains it",
    label = "not a parameter set",
    note = "`#[param(flatten)]` declares the fields of a nested parameter set directly under \
            the parent's namespace, so it applies only to types with \
            `#[derive(ParameterSet)]`"
)]
pub trait DeclareFlattened<M = Writable>: DeclareField<M> {
    /// Declares this set's parameters directly under `prefix`, with no namespace of its own.
    fn declare_flattened(
        node: &NodeState,
        prefix: &str,
        spec: FieldSpec<Self::Value, Self::Range>,
    ) -> Result<Self::Handle, ParameterSetError> {
        Self::declare(node, prefix, spec)
    }
}

/// Applies a field's spec to a parameter builder.
///
/// Public because [`declare_parameter_field!`] expands to a call to it, but not part of the
/// stable surface of the crate.
#[doc(hidden)]
pub fn apply_spec<'a, T: ParameterVariant>(
    builder: ParameterBuilder<'a, T>,
    spec: FieldSpec<T, T::Range>,
) -> ParameterBuilder<'a, T> {
    let mut builder = builder.range(spec.range);
    if let Some(default) = spec.default {
        builder = builder.default(default);
    }
    if !spec.description.is_empty() {
        builder = builder.description(spec.description);
    }
    if !spec.constraints.is_empty() {
        builder = builder.constraints(spec.constraints);
    }
    if spec.ignore_override {
        builder = builder.ignore_override();
    }
    if spec.discard_mismatching_prior_value {
        builder = builder.discard_mismatching_prior_value();
    }
    if let Some(validate) = spec.validate {
        builder = builder.validate(validate);
    }
    if let Some(discriminate) = spec.discriminate {
        builder = builder.discriminate(discriminate);
    }
    builder
}

/// An `Option<T>` field is an optional parameter: one that may have no value at all.
///
/// This is implemented once for every [`ParameterVariant`], rather than per type by
/// [`declare_parameter_field!`], because the orphan rules would not allow a crate that defines
/// its own parameter type to implement a trait from rclrs for `Option<its own type>`.
///
/// Note that `Option` of a *nested parameter set* is deliberately not implemented: ROS 2
/// parameters are declared individually, so a whole group of them cannot be absent.
impl<T: ParameterVariant> DeclareField<Writable> for Option<T> {
    type Value = T;
    type Handle = crate::OptionalParameter<T>;
    type Range = <T as ParameterVariant>::Range;

    fn declare(
        node: &NodeState,
        name: &str,
        spec: FieldSpec<Self::Value, Self::Range>,
    ) -> Result<Self::Handle, ParameterSetError> {
        apply_spec(node.declare_parameter::<T>(name), spec)
            .optional()
            .map_err(|e| ParameterSetError::new(name, e))
    }

    fn snapshot(handle: &Self::Handle) -> Self {
        handle.get()
    }

    fn into_default(self) -> Option<Self::Value> {
        self
    }
}

/// Implements [`DeclareField`] for a [`ParameterVariant`], so that it can be used as the type of
/// a [`ParameterSet`] field.
///
/// Two implementations are produced: the type becomes a mandatory parameter, and in
/// [`ReadOnly`] mode a read-only one. `Option<T>` is handled generically for every
/// [`ParameterVariant`], so it needs nothing here.
///
/// rclrs uses this for its own parameter types. Call it for a type of your own that implements
/// [`ParameterVariant`] by hand.
///
/// ```
/// use rclrs::*;
/// use std::sync::Arc;
///
/// #[derive(Clone, Debug, PartialEq)]
/// struct Hostname(Arc<str>);
///
/// impl From<Hostname> for ParameterValue {
///     fn from(value: Hostname) -> Self {
///         ParameterValue::String(value.0)
///     }
/// }
///
/// impl TryFrom<ParameterValue> for Hostname {
///     type Error = ParameterValueError;
///     fn try_from(value: ParameterValue) -> Result<Self, Self::Error> {
///         match value {
///             ParameterValue::String(v) if !v.is_empty() => Ok(Hostname(v)),
///             ParameterValue::String(_) => {
///                 Err(ParameterValueError::Invalid("hostname must not be empty".into()))
///             }
///             _ => Err(ParameterValueError::TypeMismatch),
///         }
///     }
/// }
///
/// impl ParameterVariant for Hostname {
///     type Range = ();
///     fn kind() -> ParameterKind {
///         ParameterKind::String
///     }
/// }
///
/// declare_parameter_field!(Hostname);
///
/// // `Hostname` is now usable as a parameter set field, including as an `Option`.
/// #[derive(ParameterSet)]
/// struct Config {
///     #[param(default = Hostname("localhost".into()))]
///     host: Hostname,
///     fallback: Option<Hostname>,
/// }
/// ```
#[macro_export]
macro_rules! declare_parameter_field {
    ($($t:ty),* $(,)?) => { $(
        impl $crate::DeclareField<$crate::Writable> for $t {
            type Value = $t;
            type Handle = $crate::MandatoryParameter<$t>;
            type Range = <$t as $crate::ParameterVariant>::Range;

            fn declare(
                node: &$crate::NodeState,
                name: &str,
                spec: $crate::FieldSpec<Self::Value, Self::Range>,
            ) -> ::core::result::Result<Self::Handle, $crate::ParameterSetError> {
                $crate::apply_spec(node.declare_parameter::<$t>(name), spec)
                    .mandatory()
                    .map_err(|e| $crate::ParameterSetError::new(name, e))
            }

            fn snapshot(handle: &Self::Handle) -> Self {
                handle.get()
            }

            fn into_default(self) -> ::core::option::Option<Self::Value> {
                ::core::option::Option::Some(self)
            }
        }

        impl $crate::DeclareField<$crate::ReadOnly> for $t {
            type Value = $t;
            type Handle = $crate::ReadOnlyParameter<$t>;
            type Range = <$t as $crate::ParameterVariant>::Range;

            fn declare(
                node: &$crate::NodeState,
                name: &str,
                spec: $crate::FieldSpec<Self::Value, Self::Range>,
            ) -> ::core::result::Result<Self::Handle, $crate::ParameterSetError> {
                $crate::apply_spec(node.declare_parameter::<$t>(name), spec)
                    .read_only()
                    .map_err(|e| $crate::ParameterSetError::new(name, e))
            }

            fn snapshot(handle: &Self::Handle) -> Self {
                handle.get()
            }

            fn into_default(self) -> ::core::option::Option<Self::Value> {
                ::core::option::Option::Some(self)
            }
        }
    )* };
}

declare_parameter_field!(
    // The types that represent the ROS 2 parameter types directly.
    bool,
    i64,
    f64,
    Arc<str>,
    Arc<[u8]>,
    Arc<[bool]>,
    Arc<[i64]>,
    Arc<[f64]>,
    Arc<[Arc<str>]>,
    // A dynamically typed parameter.
    ParameterValue,
    // The wider set of Rust types from `std_types`.
    String,
    PathBuf,
    Vec<u8>,
    Vec<bool>,
    Vec<i64>,
    Vec<f64>,
    Vec<String>,
    f32,
    i8,
    i16,
    i32,
    u8,
    u16,
    u32,
    // The array forms of the types above, which use whichever ROS 2 array type holds the
    // scalar's representation.
    Vec<PathBuf>,
    Vec<f32>,
    Vec<i8>,
    Vec<i16>,
    Vec<i32>,
    Vec<u16>,
    Vec<u32>,
);

#[cfg(test)]
#[path = "set_tests.rs"]
mod tests;

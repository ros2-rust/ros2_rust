mod override_map;
mod range;
mod service;
mod value;

pub(crate) use override_map::*;
pub use range::*;
use service::*;
pub use value::*;

use crate::vendor::rcl_interfaces::msg::rmw::{ParameterType, ParameterValue as RmwParameterValue};

use crate::{
    call_string_getter_with_rcl_node, rcl_bindings::*, Node, RclrsError, ENTITY_LIFECYCLE_MUTEX,
};
use std::{
    collections::{btree_map::Entry, BTreeMap},
    fmt::Debug,
    marker::PhantomData,
    sync::{Arc, Mutex, RwLock, Weak},
};

// This module implements the core logic of parameters in rclrs.
// The implementation is fairly different from the existing ROS 2 client libraries. A detailed
// explanation of the core differences and why they have been implemented is available at:
// https://github.com/ros2-rust/ros2_rust/pull/332
// Among the most relevant ones:
//
// * Parameter declaration returns an object which will be the main accessor to the parameter,
//   providing getters and, except for read only parameters, setters. Object destruction will
//   undeclare the parameter.
// * Declaration uses a builder pattern to specify ranges, description, human readable constraints
//   instead of an ParameterDescriptor argument.
// * Parameters properties of read only and dynamic are embedded in their type rather than being a
//   boolean parameter.
// * There are no runtime exceptions for common cases such as undeclared parameter, already
//   declared, or uninitialized.
// * There is no "parameter not set" type, users can instead decide to have a `Mandatory` parameter
//   that must always have a value or `Optional` parameter that can be unset.
// * Explicit API for access to undeclared parameters by having a
//   `node.use_undeclared_parameters()` API that allows access to all parameters.

#[derive(Clone, Debug)]
struct ParameterOptionsStorage {
    description: Arc<str>,
    constraints: Arc<str>,
    ranges: ParameterRanges,
}

impl<T: ParameterVariant> From<ParameterOptions<T>> for ParameterOptionsStorage {
    fn from(opts: ParameterOptions<T>) -> Self {
        Self {
            description: opts.description,
            constraints: opts.constraints,
            ranges: opts.ranges.into(),
        }
    }
}

/// Options that can be attached to a parameter, such as description, ranges.
/// Some of this data will be used to populate the ParameterDescriptor
#[derive(Clone, Debug)]
pub struct ParameterOptions<T: ParameterVariant> {
    description: Arc<str>,
    constraints: Arc<str>,
    ranges: T::Range,
}

impl<T: ParameterVariant> Default for ParameterOptions<T> {
    fn default() -> Self {
        Self {
            description: Arc::from(""),
            constraints: Arc::from(""),
            ranges: Default::default(),
        }
    }
}

#[derive(Clone, Debug)]
enum DeclaredValue {
    Mandatory(Arc<RwLock<ParameterValue>>),
    Optional(Arc<RwLock<Option<ParameterValue>>>),
    ReadOnly(ParameterValue),
}

/// Builder used to declare a parameter. Obtain this by calling
/// [`crate::Node::declare_parameter`].
#[must_use]
pub struct ParameterBuilder<'a, T: ParameterVariant> {
    name: Arc<str>,
    default_value: Option<T>,
    ignore_override: bool,
    discard_mismatching_prior_value: bool,
    discriminator: DiscriminatorFunction<'a, T>,
    options: ParameterOptions<T>,
    interface: &'a ParameterInterface,
}

impl<'a, T: ParameterVariant> ParameterBuilder<'a, T> {
    /// Sets the default value for the parameter. The parameter value will be
    /// initialized to this if no command line override was given for this
    /// parameter and if the parameter also had no value prior to being
    /// declared.
    ///
    /// To customize how the initial value of the parameter is chosen, you can
    /// provide a custom function with the method [`Self::discriminate()`]. By
    /// default, the initial value will be chosen as
    /// `default_value < override_value < prior_value` in order of increasing
    /// preference.
    pub fn default(mut self, value: T) -> Self {
        self.default_value = Some(value);
        self
    }

    /// Ignore any override that was given for this parameter.
    ///
    /// If you also use [`Self::discriminate()`], the
    /// [`AvailableValues::override_value`] field given to the discriminator
    /// will be [`None`] even if the user had provided an override.
    pub fn ignore_override(mut self) -> Self {
        self.ignore_override = true;
        self
    }

    /// If the parameter was set to a value before being declared with a type
    /// that does not match this declaration, discard the prior value instead
    /// of emitting a [`DeclarationError::PriorValueTypeMismatch`].
    ///
    /// If the type of the prior value does match the declaration, it will
    /// still be provided to the discriminator.
    pub fn discard_mismatching_prior_value(mut self) -> Self {
        self.discard_mismatching_prior_value = true;
        self
    }

    /// Decide what the initial value for the parameter will be based on the
    /// available `default_value`, `override_value`, or `prior_value`.
    ///
    /// The default discriminator is [`default_initial_value_discriminator()`].
    pub fn discriminate<F>(mut self, f: F) -> Self
    where
        F: FnOnce(AvailableValues<T>) -> Option<T> + 'a,
    {
        self.discriminator = Box::new(f);
        self
    }

    /// Sets the range for the parameter.
    pub fn range(mut self, range: T::Range) -> Self {
        self.options.ranges = range;
        self
    }

    /// Sets the parameter's human readable description.
    pub fn description(mut self, description: impl Into<Arc<str>>) -> Self {
        self.options.description = description.into();
        self
    }

    /// Sets the parameter's human readable constraints.
    /// These are not enforced by the library but are displayed on parameter description requests
    /// and can be used by integrators to understand complex constraints.
    pub fn constraints(mut self, constraints: impl Into<Arc<str>>) -> Self {
        self.options.constraints = constraints.into();
        self
    }

    /// Declares the parameter as a Mandatory parameter, that must always have a value.
    ///
    /// ## See also
    /// * [`Self::optional()`]
    /// * [`Self::read_only()`]
    pub fn mandatory(self) -> Result<MandatoryParameter<T>, DeclarationError> {
        self.try_into()
    }

    /// Declares the parameter as a ReadOnly parameter, that cannot be edited.
    ///
    /// # See also
    /// * [`Self::optional()`]
    /// * [`Self::mandatory()`]
    pub fn read_only(self) -> Result<ReadOnlyParameter<T>, DeclarationError> {
        self.try_into()
    }

    /// Declares the parameter as an Optional parameter, that can be unset.
    ///
    /// This will never return the [`DeclarationError::NoValueAvailable`] variant.
    ///
    /// ## See also
    /// * [`Self::mandatory()`]
    /// * [`Self::read_only()`]
    pub fn optional(self) -> Result<OptionalParameter<T>, DeclarationError> {
        self.try_into()
    }
}

impl<'a, T> ParameterBuilder<'a, Arc<[T]>>
where
    Arc<[T]>: ParameterVariant,
{
    /// Sets the default for an array-like parameter from an iterable.
    pub fn default_from_iter(mut self, default_value: impl IntoIterator<Item = T>) -> Self {
        self.default_value = Some(default_value.into_iter().collect());
        self
    }
}

impl<'a> ParameterBuilder<'a, Arc<[Arc<str>]>> {
    /// Sets the default for the parameter from a string-like array.
    pub fn default_string_array<U>(mut self, default_value: U) -> Self
    where
        U: IntoIterator,
        U::Item: Into<Arc<str>>,
    {
        self.default_value = Some(default_value.into_iter().map(|v| v.into()).collect());
        self
    }
}

/// This struct is given to the discriminator function of the
/// [`ParameterBuilder`] so it knows what values are available to choose from.
pub struct AvailableValues<'a, T> {
    /// The value given to the parameter builder as the default value.
    pub default_value: Option<T>,
    /// The value given as an override value, usually as a command line argument.
    pub override_value: Option<T>,
    /// A prior value that the parameter was set to before it was declared.
    pub prior_value: Option<T>,
    /// The valid ranges for the parameter value.
    pub ranges: &'a ParameterRanges,
}

/// The default discriminator that chooses the initial value for a parameter.
/// The implementation here uses a simple preference of
/// ```notrust
/// default_value < override_value < prior_value
/// ```
/// in ascending order of preference.
///
/// The `prior_value` will automatically be discarded if it is outside the
/// designated range. The override value will not be discarded if it is out of
/// range because that is more likely to be an error that needs to be escalated.
/// You can replace all of this with custom behavior by providing your own
/// discriminator function to [`ParameterBuilder::discriminate()`].
pub fn default_initial_value_discriminator<T: ParameterVariant>(
    available: AvailableValues<T>,
) -> Option<T> {
    if let Some(prior) = available.prior_value {
        if available.ranges.in_range(&prior.clone().into()) {
            return Some(prior);
        }
    }
    if available.override_value.is_some() {
        return available.override_value;
    }
    available.default_value
}

type DiscriminatorFunction<'a, T> = Box<dyn FnOnce(AvailableValues<T>) -> Option<T> + 'a>;

impl<T: ParameterVariant> TryFrom<ParameterBuilder<'_, T>> for OptionalParameter<T> {
    type Error = DeclarationError;

    fn try_from(builder: ParameterBuilder<T>) -> Result<Self, Self::Error> {
        let ranges = builder.options.ranges.clone().into();
        let initial_value = builder.interface.get_declaration_initial_value::<T>(
            &builder.name,
            builder.default_value,
            builder.ignore_override,
            builder.discard_mismatching_prior_value,
            builder.discriminator,
            &ranges,
        )?;
        let value = Arc::new(RwLock::new(initial_value.map(|v| v.into())));
        builder.interface.store_parameter(
            builder.name.clone(),
            T::kind(),
            DeclaredValue::Optional(value.clone()),
            builder.options.into(),
        );
        Ok(OptionalParameter {
            name: builder.name,
            value,
            ranges,
            map: Arc::downgrade(&builder.interface.parameter_map),
            _marker: Default::default(),
        })
    }
}

/// A parameter that must have a value
/// This struct has ownership of the declared parameter. Additional parameter declaration will fail
/// while this struct exists and the parameter will be undeclared when it is dropped.
pub struct MandatoryParameter<T: ParameterVariant> {
    name: Arc<str>,
    value: Arc<RwLock<ParameterValue>>,
    ranges: ParameterRanges,
    map: Weak<Mutex<ParameterMap>>,
    _marker: PhantomData<T>,
}

impl<T: ParameterVariant + Debug> Debug for MandatoryParameter<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MandatoryParameter")
            .field("name", &self.name)
            .field("value", &self.get())
            .field("range", &self.ranges)
            .finish()
    }
}

impl<T: ParameterVariant> Drop for MandatoryParameter<T> {
    fn drop(&mut self) {
        // Clear the entry from the parameter map
        if let Some(map) = self.map.upgrade() {
            let storage = &mut map.lock().unwrap().storage;
            storage.remove(&self.name);
        }
    }
}

impl<'a, T: ParameterVariant + 'a> TryFrom<ParameterBuilder<'a, T>> for MandatoryParameter<T> {
    type Error = DeclarationError;

    fn try_from(builder: ParameterBuilder<T>) -> Result<Self, Self::Error> {
        let ranges = builder.options.ranges.clone().into();
        let initial_value = builder.interface.get_declaration_initial_value::<T>(
            &builder.name,
            builder.default_value,
            builder.ignore_override,
            builder.discard_mismatching_prior_value,
            builder.discriminator,
            &ranges,
        )?;
        let Some(initial_value) = initial_value else {
            return Err(DeclarationError::NoValueAvailable);
        };
        let value = Arc::new(RwLock::new(initial_value.into()));
        builder.interface.store_parameter(
            builder.name.clone(),
            T::kind(),
            DeclaredValue::Mandatory(value.clone()),
            builder.options.into(),
        );
        Ok(MandatoryParameter {
            name: builder.name,
            value,
            ranges,
            map: Arc::downgrade(&builder.interface.parameter_map),
            _marker: Default::default(),
        })
    }
}

/// A parameter that might not have a value, represented by `Option<T>`.
/// This struct has ownership of the declared parameter. Additional parameter declaration will fail
/// while this struct exists and the parameter will be undeclared when it is dropped.
pub struct OptionalParameter<T: ParameterVariant> {
    name: Arc<str>,
    value: Arc<RwLock<Option<ParameterValue>>>,
    ranges: ParameterRanges,
    map: Weak<Mutex<ParameterMap>>,
    _marker: PhantomData<T>,
}

impl<T: ParameterVariant + Debug> Debug for OptionalParameter<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("OptionalParameter")
            .field("name", &self.name)
            .field("value", &self.get())
            .field("range", &self.ranges)
            .finish()
    }
}

impl<T: ParameterVariant> Drop for OptionalParameter<T> {
    fn drop(&mut self) {
        // Clear the entry from the parameter map
        if let Some(map) = self.map.upgrade() {
            let storage = &mut map.lock().unwrap().storage;
            storage.remove(&self.name);
        }
    }
}

/// A parameter that must have a value and cannot be written to
/// This struct has ownership of the declared parameter. Additional parameter declaration will fail
/// while this struct exists and the parameter will be undeclared when it is dropped.
pub struct ReadOnlyParameter<T: ParameterVariant> {
    name: Arc<str>,
    value: ParameterValue,
    map: Weak<Mutex<ParameterMap>>,
    _marker: PhantomData<T>,
}

impl<T: ParameterVariant + Debug> Debug for ReadOnlyParameter<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ReadOnlyParameter")
            .field("name", &self.name)
            .field("value", &self.value)
            .finish()
    }
}

impl<T: ParameterVariant> Drop for ReadOnlyParameter<T> {
    fn drop(&mut self) {
        // Clear the entry from the parameter map
        if let Some(map) = self.map.upgrade() {
            let storage = &mut map.lock().unwrap().storage;
            storage.remove(&self.name);
        }
    }
}

impl<'a, T: ParameterVariant + 'a> TryFrom<ParameterBuilder<'a, T>> for ReadOnlyParameter<T> {
    type Error = DeclarationError;

    fn try_from(builder: ParameterBuilder<T>) -> Result<Self, Self::Error> {
        let ranges = builder.options.ranges.clone().into();
        let initial_value = builder.interface.get_declaration_initial_value::<T>(
            &builder.name,
            builder.default_value,
            builder.ignore_override,
            builder.discard_mismatching_prior_value,
            builder.discriminator,
            &ranges,
        )?;
        let Some(initial_value) = initial_value else {
            return Err(DeclarationError::NoValueAvailable);
        };
        let value = initial_value.into();
        builder.interface.store_parameter(
            builder.name.clone(),
            T::kind(),
            DeclaredValue::ReadOnly(value.clone()),
            builder.options.into(),
        );
        Ok(ReadOnlyParameter {
            name: builder.name,
            value,
            map: Arc::downgrade(&builder.interface.parameter_map),
            _marker: Default::default(),
        })
    }
}

#[derive(Clone, Debug)]
struct DeclaredStorage {
    value: DeclaredValue,
    kind: ParameterKind,
    options: ParameterOptionsStorage,
}

#[derive(Debug)]
enum ParameterStorage {
    Declared(DeclaredStorage),
    Undeclared(ParameterValue),
}

impl ParameterStorage {
    pub(crate) fn to_parameter_type(&self) -> u8 {
        match self {
            ParameterStorage::Declared(s) => match s.kind {
                ParameterKind::Bool => ParameterType::PARAMETER_BOOL,
                ParameterKind::Integer => ParameterType::PARAMETER_INTEGER,
                ParameterKind::Double => ParameterType::PARAMETER_DOUBLE,
                ParameterKind::String => ParameterType::PARAMETER_STRING,
                ParameterKind::ByteArray => ParameterType::PARAMETER_BYTE_ARRAY,
                ParameterKind::BoolArray => ParameterType::PARAMETER_BOOL_ARRAY,
                ParameterKind::IntegerArray => ParameterType::PARAMETER_INTEGER_ARRAY,
                ParameterKind::DoubleArray => ParameterType::PARAMETER_DOUBLE_ARRAY,
                ParameterKind::StringArray => ParameterType::PARAMETER_STRING_ARRAY,
                ParameterKind::Dynamic => match &s.value {
                    // Unwraps here are safe because None will only be returned if the RwLock is
                    // poisoned, but it is only written in internal set(value) calls that have no
                    // way to panic.
                    DeclaredValue::Mandatory(v) => v.read().unwrap().rcl_parameter_type(),
                    DeclaredValue::Optional(v) => v
                        .read()
                        .unwrap()
                        .as_ref()
                        .map(|v| v.rcl_parameter_type())
                        .unwrap_or(ParameterType::PARAMETER_NOT_SET),
                    DeclaredValue::ReadOnly(v) => v.rcl_parameter_type(),
                },
            },
            ParameterStorage::Undeclared(value) => value.rcl_parameter_type(),
        }
    }
}

#[derive(Debug, Default)]
pub(crate) struct ParameterMap {
    storage: BTreeMap<Arc<str>, ParameterStorage>,
    allow_undeclared: bool,
}

impl ParameterMap {
    /// Validates the requested parameter setting and returns an error if the requested value is
    /// not valid.
    fn validate_parameter_setting(
        &self,
        name: &str,
        value: RmwParameterValue,
    ) -> Result<ParameterValue, &str> {
        let Ok(value): Result<ParameterValue, _> = value.try_into() else {
            return Err("Invalid parameter type");
        };
        match self.storage.get(name) {
            Some(entry) => {
                if let ParameterStorage::Declared(storage) = entry {
                    if std::mem::discriminant(&storage.kind)
                        == std::mem::discriminant(&value.kind())
                        || matches!(storage.kind, ParameterKind::Dynamic)
                    {
                        if !storage.options.ranges.in_range(&value) {
                            return Err("Parameter value is out of range");
                        }
                        if matches!(&storage.value, DeclaredValue::ReadOnly(_)) {
                            return Err("Parameter is read only");
                        }
                    } else {
                        return Err(
                            "Parameter set to different type and dynamic typing is disabled",
                        );
                    }
                }
            }
            None => {
                if !self.allow_undeclared {
                    return Err(
                        "Parameter was not declared and undeclared parameters are not allowed",
                    );
                }
            }
        }
        Ok(value)
    }

    /// Stores the requested parameter in the map.
    fn store_parameter(&mut self, name: Arc<str>, value: ParameterValue) {
        match self.storage.entry(name) {
            Entry::Occupied(mut entry) => match entry.get_mut() {
                ParameterStorage::Declared(storage) => match &storage.value {
                    DeclaredValue::Mandatory(p) => *p.write().unwrap() = value,
                    DeclaredValue::Optional(p) => *p.write().unwrap() = Some(value),
                    DeclaredValue::ReadOnly(_) => unreachable!(),
                },
                ParameterStorage::Undeclared(param) => {
                    *param = value;
                }
            },
            Entry::Vacant(entry) => {
                entry.insert(ParameterStorage::Undeclared(value));
            }
        }
    }
}

impl<T: ParameterVariant> MandatoryParameter<T> {
    /// Returns a clone of the most recent value of the parameter.
    pub fn get(&self) -> T {
        self.value.read().unwrap().clone().try_into().ok().unwrap()
    }

    /// Sets the parameter value.
    /// Returns [`ParameterValueError::OutOfRange`] if the value is out of the parameter's range.
    pub fn set<U: Into<T>>(&self, value: U) -> Result<(), ParameterValueError> {
        let value = value.into().into();
        if !self.ranges.in_range(&value) {
            return Err(ParameterValueError::OutOfRange);
        }
        *self.value.write().unwrap() = value;
        Ok(())
    }
}

impl<T: ParameterVariant> ReadOnlyParameter<T> {
    /// Returns a clone of the most recent value of the parameter.
    pub fn get(&self) -> T {
        self.value.clone().try_into().ok().unwrap()
    }
}

impl<T: ParameterVariant> OptionalParameter<T> {
    /// Returns a clone of the most recent value of the parameter.
    pub fn get(&self) -> Option<T> {
        self.value
            .read()
            .unwrap()
            .clone()
            .map(|p| p.try_into().ok().unwrap())
    }

    /// Assigns a value to the optional parameter, setting it to `Some(value)`.
    /// Returns [`ParameterValueError::OutOfRange`] if the value is out of the parameter's range.
    pub fn set<U: Into<T>>(&self, value: U) -> Result<(), ParameterValueError> {
        let value = value.into().into();
        if !self.ranges.in_range(&value) {
            return Err(ParameterValueError::OutOfRange);
        }
        *self.value.write().unwrap() = Some(value);
        Ok(())
    }

    /// Unsets the optional parameter value to `None`.
    pub fn unset(&self) {
        *self.value.write().unwrap() = None;
    }
}

/// Allows access to all parameters via get / set functions, using their name as a key.
pub struct Parameters<'a> {
    pub(crate) interface: &'a ParameterInterface,
}

/// Describes errors that can be generated when trying to set a parameter's value.
#[derive(Debug)]
pub enum ParameterValueError {
    /// Parameter value was out of the parameter's range.
    OutOfRange,
    /// Parameter was stored in a static type and an operation on a different type was attempted.
    TypeMismatch,
    /// A write on a read-only parameter was attempted.
    ReadOnly,
}

impl std::fmt::Display for ParameterValueError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ParameterValueError::OutOfRange => write!(f, "parameter value was out of the parameter's range"),
            ParameterValueError::TypeMismatch => write!(f, "parameter was stored in a static type and an operation on a different type was attempted"),
            ParameterValueError::ReadOnly => write!(f, "a write on a read-only parameter was attempted"),
        }
    }
}

impl std::error::Error for ParameterValueError {}

/// Error that can be generated when doing operations on parameters.
#[derive(Debug)]
pub enum DeclarationError {
    /// Parameter was already declared and a new declaration was attempted.
    AlreadyDeclared,
    /// Parameter was declared as non optional but no value was available, either through a user
    /// specified default, a command-line override, or a previously set value.
    NoValueAvailable,
    /// The override value that was provided has the wrong type. This error is bypassed
    /// when using [`ParameterBuilder::ignore_override()`].
    OverrideValueTypeMismatch,
    /// The value that the parameter was already set to has the wrong type. This error
    /// is bypassed when using [`ParameterBuilder::discard_mismatching_prior_value`].
    PriorValueTypeMismatch,
    /// The initial value that was selected is out of range.
    InitialValueOutOfRange,
    /// An invalid range was provided to a parameter declaration (i.e. lower bound > higher bound).
    InvalidRange,
}

impl std::fmt::Display for DeclarationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DeclarationError::AlreadyDeclared => write!(f, "parameter was already declared, but a new declaration was attempted"),
            DeclarationError::NoValueAvailable => {
                write!(f, "parameter was declared as non-optional but no value was available, either through a user specified default, a command-line override, or a previously set value")
            },
            DeclarationError::OverrideValueTypeMismatch => {
                write!(f, "the override value that was provided has the wrong type")
            },
            DeclarationError::PriorValueTypeMismatch => {
                write!(f, "the value that the parameter was already set to has the wrong type")
            },
            DeclarationError::InitialValueOutOfRange => write!(f, "the initial value that was selected is out of range"),
            DeclarationError::InvalidRange => write!(f, "an invalid range was provided to a parameter declaration (i.e. lower bound > higher bound)"),
        }
    }
}

impl std::error::Error for DeclarationError {}

impl<'a> Parameters<'a> {
    /// Tries to read a parameter of the requested type.
    ///
    /// Returns `Some(T)` if a parameter of the requested type exists, `None` otherwise.
    pub fn get<T: ParameterVariant>(&self, name: &str) -> Option<T> {
        let storage = &self.interface.parameter_map.lock().unwrap().storage;
        let storage = storage.get(name)?;
        match storage {
            ParameterStorage::Declared(storage) => match &storage.value {
                DeclaredValue::Mandatory(p) => p.read().unwrap().clone().try_into().ok(),
                DeclaredValue::Optional(p) => {
                    p.read().unwrap().clone().and_then(|p| p.try_into().ok())
                }
                DeclaredValue::ReadOnly(p) => p.clone().try_into().ok(),
            },
            ParameterStorage::Undeclared(value) => value.clone().try_into().ok(),
        }
    }

    /// Tries to set a parameter with the requested value.
    ///
    /// Returns:
    /// * `Ok(())` if setting was successful.
    /// * [`Err(DeclarationError::TypeMismatch)`] if the type of the requested value is different
    ///   from the parameter's type.
    /// * [`Err(DeclarationError::OutOfRange)`] if the requested value is out of the parameter's
    ///   range.
    /// * [`Err(DeclarationError::ReadOnly)`] if the parameter is read only.
    pub fn set<T: ParameterVariant>(
        &self,
        name: impl Into<Arc<str>>,
        value: T,
    ) -> Result<(), ParameterValueError> {
        let mut map = self.interface.parameter_map.lock().unwrap();
        let name: Arc<str> = name.into();
        match map.storage.entry(name) {
            Entry::Occupied(mut entry) => {
                // If it's declared, we can only set if it's the same variant.
                // Undeclared parameters are dynamic by default
                match entry.get_mut() {
                    ParameterStorage::Declared(param) => {
                        if T::kind() == param.kind {
                            let value = value.into();
                            if !param.options.ranges.in_range(&value) {
                                return Err(ParameterValueError::OutOfRange);
                            }
                            match &param.value {
                                DeclaredValue::Mandatory(p) => *p.write().unwrap() = value,
                                DeclaredValue::Optional(p) => *p.write().unwrap() = Some(value),
                                DeclaredValue::ReadOnly(_) => {
                                    return Err(ParameterValueError::ReadOnly);
                                }
                            }
                        } else {
                            return Err(ParameterValueError::TypeMismatch);
                        }
                    }
                    ParameterStorage::Undeclared(param) => {
                        *param = value.into();
                    }
                }
            }
            Entry::Vacant(entry) => {
                entry.insert(ParameterStorage::Undeclared(value.into()));
            }
        }
        Ok(())
    }
}

pub(crate) struct ParameterInterface {
    parameter_map: Arc<Mutex<ParameterMap>>,
    override_map: ParameterOverrideMap,
    services: Mutex<Option<ParameterService>>,
}

impl ParameterInterface {
    pub(crate) fn new(
        rcl_node: &rcl_node_t,
        node_arguments: &rcl_arguments_t,
        global_arguments: &rcl_arguments_t,
    ) -> Result<Self, RclrsError> {
        let override_map = unsafe {
            let _lifecycle_lock = ENTITY_LIFECYCLE_MUTEX.lock().unwrap();
            let fqn = call_string_getter_with_rcl_node(rcl_node, rcl_node_get_fully_qualified_name);
            resolve_parameter_overrides(&fqn, node_arguments, global_arguments)?
        };

        Ok(ParameterInterface {
            parameter_map: Default::default(),
            override_map,
            services: Mutex::new(None),
        })
    }

    pub(crate) fn declare<'a, T: ParameterVariant + 'a>(
        &'a self,
        name: Arc<str>,
    ) -> ParameterBuilder<'a, T> {
        ParameterBuilder {
            name,
            default_value: None,
            ignore_override: false,
            discard_mismatching_prior_value: false,
            discriminator: Box::new(default_initial_value_discriminator::<T>),
            options: Default::default(),
            interface: self,
        }
    }

    pub(crate) fn create_services(&self, node: &Node) -> Result<(), RclrsError> {
        *self.services.lock().unwrap() =
            Some(ParameterService::new(node, self.parameter_map.clone())?);
        Ok(())
    }

    fn get_declaration_initial_value<'a, T: ParameterVariant + 'a>(
        &self,
        name: &str,
        default_value: Option<T>,
        ignore_override: bool,
        discard_mismatching_prior: bool,
        discriminator: DiscriminatorFunction<T>,
        ranges: &ParameterRanges,
    ) -> Result<Option<T>, DeclarationError> {
        ranges.validate()?;
        let override_value: Option<T> = if ignore_override {
            None
        } else if let Some(override_value) = self.override_map.get(name).cloned() {
            Some(
                override_value
                    .try_into()
                    .map_err(|_| DeclarationError::OverrideValueTypeMismatch)?,
            )
        } else {
            None
        };

        let prior_value =
            if let Some(prior_value) = self.parameter_map.lock().unwrap().storage.get(name) {
                match prior_value {
                    ParameterStorage::Declared(_) => return Err(DeclarationError::AlreadyDeclared),
                    ParameterStorage::Undeclared(param) => match param.clone().try_into() {
                        Ok(prior) => Some(prior),
                        Err(_) => {
                            if !discard_mismatching_prior {
                                return Err(DeclarationError::PriorValueTypeMismatch);
                            }
                            None
                        }
                    },
                }
            } else {
                None
            };

        let selection = discriminator(AvailableValues {
            default_value,
            override_value,
            prior_value,
            ranges,
        });
        if let Some(initial_value) = &selection {
            if !ranges.in_range(&initial_value.clone().into()) {
                return Err(DeclarationError::InitialValueOutOfRange);
            }
        }
        Ok(selection)
    }

    fn store_parameter(
        &self,
        name: Arc<str>,
        kind: ParameterKind,
        value: DeclaredValue,
        options: ParameterOptionsStorage,
    ) {
        self.parameter_map.lock().unwrap().storage.insert(
            name,
            ParameterStorage::Declared(DeclaredStorage {
                options,
                value,
                kind,
            }),
        );
    }

    pub(crate) fn allow_undeclared(&self) {
        self.parameter_map.lock().unwrap().allow_undeclared = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{create_node, Context};

    #[test]
    fn test_parameter_override_errors() {
        // Create a new node with a few parameter overrides
        let ctx = Context::new([
            String::from("--ros-args"),
            String::from("-p"),
            String::from("declared_int:=10"),
        ])
        .unwrap();
        let node = create_node(&ctx, &format!("param_test_node_{}", line!())).unwrap();

        // Declaring a parameter with a different type than what was overridden should return an
        // error
        assert!(matches!(
            node.declare_parameter("declared_int")
                .default(1.0)
                .mandatory(),
            Err(DeclarationError::OverrideValueTypeMismatch)
        ));

        // The error should not happen if we ignore overrides
        assert!(node
            .declare_parameter("declared_int")
            .default(1.0)
            .ignore_override()
            .mandatory()
            .is_ok());

        // If the override does not respect the range, we should return an error
        let range = ParameterRange {
            upper: Some(5),
            ..Default::default()
        };
        assert!(matches!(
            node.declare_parameter("declared_int")
                .default(1)
                .range(range.clone())
                .mandatory(),
            Err(DeclarationError::InitialValueOutOfRange)
        ));

        // The override being out of range should not matter if we use
        // ignore_override
        assert!(node
            .declare_parameter("declared_int")
            .default(1)
            .range(range)
            .ignore_override()
            .mandatory()
            .is_ok());
    }

    #[test]
    fn test_parameter_setting_declaring() {
        // Create a new node with a few parameter overrides
        let ctx = Context::new([
            String::from("--ros-args"),
            String::from("-p"),
            String::from("declared_int:=10"),
            String::from("-p"),
            String::from("double_array:=[1.0, 2.0]"),
            String::from("-p"),
            String::from("optional_bool:=true"),
            String::from("-p"),
            String::from("non_declared_string:='param'"),
        ])
        .unwrap();
        let node = create_node(&ctx, &format!("param_test_node_{}", line!())).unwrap();

        let overridden_int = node
            .declare_parameter("declared_int")
            .default(123)
            .mandatory()
            .unwrap();
        assert_eq!(overridden_int.get(), 10);

        let new_param = node
            .declare_parameter("new_param")
            .default(2.0)
            .mandatory()
            .unwrap();
        assert_eq!(new_param.get(), 2.0);

        // Getting a parameter that was declared should work
        assert_eq!(
            node.use_undeclared_parameters().get::<f64>("new_param"),
            Some(2.0)
        );

        // Getting / Setting a parameter with the wrong type should not work
        assert!(node
            .use_undeclared_parameters()
            .get::<i64>("new_param")
            .is_none());
        assert!(matches!(
            node.use_undeclared_parameters().set("new_param", 42),
            Err(ParameterValueError::TypeMismatch)
        ));

        // Setting a parameter should update both existing parameter objects and be reflected in
        // new node.use_undeclared_parameters().get() calls
        assert!(node
            .use_undeclared_parameters()
            .set("new_param", 10.0)
            .is_ok());
        assert_eq!(
            node.use_undeclared_parameters().get("new_param"),
            Some(10.0)
        );
        assert_eq!(new_param.get(), 10.0);
        new_param.set(5.0).unwrap();
        assert_eq!(new_param.get(), 5.0);
        assert_eq!(node.use_undeclared_parameters().get("new_param"), Some(5.0));

        // Getting a parameter that was not declared should not work
        assert_eq!(
            node.use_undeclared_parameters()
                .get::<f64>("non_existing_param"),
            None
        );

        // Getting a parameter that was not declared should not work, even if a value was provided
        // as a parameter override
        assert_eq!(
            node.use_undeclared_parameters()
                .get::<Arc<str>>("non_declared_string"),
            None
        );

        // If a param is set when undeclared, the following declared value should have the
        // previously set value.
        {
            node.use_undeclared_parameters()
                .set("new_bool", true)
                .unwrap();
            let bool_param = node
                .declare_parameter("new_bool")
                .default(false)
                .mandatory()
                .unwrap();
            assert!(bool_param.get());
        }
        {
            node.use_undeclared_parameters()
                .set("new_bool", true)
                .unwrap();
            let bool_param = node
                .declare_parameter("new_bool")
                .default(false)
                .optional()
                .unwrap();
            assert_eq!(bool_param.get(), Some(true));
        }

        let optional_param = node
            .declare_parameter("non_existing_bool")
            .optional()
            .unwrap();
        assert_eq!(optional_param.get(), None);
        optional_param.set(true).unwrap();
        assert_eq!(optional_param.get(), Some(true));
        optional_param.unset();
        assert_eq!(optional_param.get(), None);

        let optional_param2 = node
            .declare_parameter("non_existing_bool2")
            .default(false)
            .optional()
            .unwrap();
        assert_eq!(optional_param2.get(), Some(false));

        // This was provided as a parameter override, hence should be set to true
        let optional_param3 = node
            .declare_parameter("optional_bool")
            .default(false)
            .optional()
            .unwrap();
        assert_eq!(optional_param3.get(), Some(true));

        // double_array was overriden to [1.0, 2.0] through command line overrides
        let array_param = node
            .declare_parameter("double_array")
            .default_from_iter(vec![10.0, 20.0])
            .mandatory()
            .unwrap();
        assert_eq!(array_param.get()[0], 1.0);
        assert_eq!(array_param.get()[1], 2.0);

        let array_param = node
            .declare_parameter("string_array")
            .default_string_array(vec!["Hello", "World"])
            .mandatory()
            .unwrap();
        assert_eq!(array_param.get()[0], "Hello".into());
        assert_eq!(array_param.get()[1], "World".into());

        // If a value is set when undeclared, the following declare_parameter should have the
        // previously set value.
        node.use_undeclared_parameters()
            .set("undeclared_int", 42)
            .unwrap();
        let undeclared_int = node
            .declare_parameter("undeclared_int")
            .default(10)
            .mandatory()
            .unwrap();
        assert_eq!(undeclared_int.get(), 42);
    }

    #[test]
    fn test_override_undeclared_set_priority() {
        let ctx = Context::new([
            String::from("--ros-args"),
            String::from("-p"),
            String::from("declared_int:=10"),
        ])
        .unwrap();
        let node = create_node(&ctx, &format!("param_test_node_{}", line!())).unwrap();
        // If a parameter was set as an override and as an undeclared parameter, the undeclared
        // value should get priority
        node.use_undeclared_parameters()
            .set("declared_int", 20)
            .unwrap();
        let param = node
            .declare_parameter("declared_int")
            .default(30)
            .mandatory()
            .unwrap();
        assert_eq!(param.get(), 20);
    }

    #[test]
    fn test_parameter_scope_redeclaring() {
        let ctx = Context::new([
            String::from("--ros-args"),
            String::from("-p"),
            String::from("declared_int:=10"),
        ])
        .unwrap();
        let node = create_node(&ctx, &format!("param_test_node_{}", line!())).unwrap();
        {
            // Setting a parameter with an override
            let param = node
                .declare_parameter("declared_int")
                .default(1)
                .mandatory()
                .unwrap();
            assert_eq!(param.get(), 10);
            param.set(2).unwrap();
            assert_eq!(param.get(), 2);
            // Redeclaring should fail
            assert!(matches!(
                node.declare_parameter("declared_int")
                    .default(1)
                    .mandatory(),
                Err(DeclarationError::AlreadyDeclared)
            ));
        }
        {
            // Parameter went out of scope, redeclaring should be OK and return command line
            // override
            let param = node
                .declare_parameter::<i64>("declared_int")
                .mandatory()
                .unwrap();
            assert_eq!(param.get(), 10);
        }
        // After a declared parameter went out of scope and was cleared, it should still be
        // possible to use it as an undeclared parameter, type can now be changed
        assert!(node
            .use_undeclared_parameters()
            .get::<i64>("declared_int")
            .is_none());
        node.use_undeclared_parameters()
            .set("declared_int", 1.0)
            .unwrap();
        assert_eq!(
            node.use_undeclared_parameters().get::<f64>("declared_int"),
            Some(1.0)
        );
    }

    #[test]
    fn test_parameter_ranges() {
        let ctx = Context::new([]).unwrap();
        let node = create_node(&ctx, &format!("param_test_node_{}", line!())).unwrap();
        // Setting invalid ranges should fail
        let range = ParameterRange {
            lower: Some(10),
            upper: Some(-10),
            step: Some(3),
        };
        assert!(matches!(
            node.declare_parameter("int_param")
                .default(5)
                .range(range)
                .mandatory(),
            Err(DeclarationError::InvalidRange)
        ));
        let range = ParameterRange {
            lower: Some(-10),
            upper: Some(10),
            step: Some(-1),
        };
        assert!(matches!(
            node.declare_parameter("int_param")
                .default(5)
                .range(range)
                .mandatory(),
            Err(DeclarationError::InvalidRange)
        ));
        // Setting parameters out of range should fail
        let range = ParameterRange {
            lower: Some(-10),
            upper: Some(10),
            step: Some(3),
        };
        assert!(matches!(
            node.declare_parameter("out_of_range_int")
                .default(100)
                .range(range.clone())
                .mandatory(),
            Err(DeclarationError::InitialValueOutOfRange)
        ));
        assert!(matches!(
            node.declare_parameter("wrong_step_int")
                .default(-9)
                .range(range.clone())
                .mandatory(),
            Err(DeclarationError::InitialValueOutOfRange)
        ));
        let param = node
            .declare_parameter("int_param")
            .default(-7)
            .range(range)
            .mandatory()
            .unwrap();
        // Out of step but equal to upper, this is OK
        assert!(param.set(10).is_ok());
        // Trying to set it as undeclared should have the same result
        assert!(matches!(
            node.use_undeclared_parameters().set("int_param", 100),
            Err(ParameterValueError::OutOfRange)
        ));
        assert!(matches!(
            node.use_undeclared_parameters().set("int_param", -9),
            Err(ParameterValueError::OutOfRange)
        ));
        assert!(node
            .use_undeclared_parameters()
            .set("int_param", -4)
            .is_ok());

        // Same for a double parameter
        let range = ParameterRange {
            lower: Some(-10.0),
            upper: Some(10.0),
            step: Some(3.0),
        };
        assert!(matches!(
            node.declare_parameter("out_of_range_double")
                .default(100.0)
                .range(range.clone())
                .optional(),
            Err(DeclarationError::InitialValueOutOfRange)
        ));
        assert!(matches!(
            node.declare_parameter("wrong_step_double")
                .default(-9.0)
                .range(range.clone())
                .read_only(),
            Err(DeclarationError::InitialValueOutOfRange)
        ));
        let param = node
            .declare_parameter("double_param")
            .default(-7.0)
            .range(range.clone())
            .mandatory()
            .unwrap();
        // Out of step but equal to upper, this is OK
        assert!(param.set(10.0).is_ok());
        // Quite close but out of tolerance, should fail
        assert!(matches!(
            param.set(-7.001),
            Err(ParameterValueError::OutOfRange)
        ));
        // Close to step within a few EPSILON, should be OK
        assert!(param.set(-7.0 - f64::EPSILON * 10.0).is_ok());
        assert!(param.set(-7.0 + f64::EPSILON * 10.0).is_ok());
        // Close to upper within a few EPSILON, should be OK
        assert!(param.set(10.0 - f64::EPSILON * 10.0).is_ok());
        assert!(param.set(10.0 + f64::EPSILON * 10.0).is_ok());
        // Close to lower within a few EPSILON, should be OK
        assert!(param.set(-10.0 - f64::EPSILON * 10.0).is_ok());
        assert!(param.set(-10.0 + f64::EPSILON * 10.0).is_ok());
        // Trying to set it as undeclared should have the same result
        assert!(matches!(
            node.use_undeclared_parameters().set("double_param", 100.0),
            Err(ParameterValueError::OutOfRange)
        ));
        assert!(matches!(
            node.use_undeclared_parameters().set("double_param", -9.0),
            Err(ParameterValueError::OutOfRange)
        ));
        assert!(node
            .use_undeclared_parameters()
            .set("double_param", -4.0)
            .is_ok());
    }

    #[test]
    fn test_readonly_parameters() {
        let ctx = Context::new([]).unwrap();
        let node = create_node(&ctx, &format!("param_test_node_{}", line!())).unwrap();
        let param = node
            .declare_parameter("int_param")
            .default(100)
            .read_only()
            .unwrap();
        // Multiple copies cannot be declared
        assert!(matches!(
            node.declare_parameter("int_param").default(100).read_only(),
            Err(DeclarationError::AlreadyDeclared)
        ));
        // A reading should work and return the correct value:w
        assert_eq!(param.get(), 100);
        assert_eq!(
            node.use_undeclared_parameters().get::<i64>("int_param"),
            Some(100)
        );
        // Setting should fail
        assert!(matches!(
            node.use_undeclared_parameters().set("int_param", 10),
            Err(ParameterValueError::ReadOnly)
        ));
    }

    #[test]
    fn test_preexisting_value_error() {
        let ctx = Context::new([]).unwrap();
        let node = create_node(&ctx, &format!("param_test_node_{}", line!())).unwrap();
        node.use_undeclared_parameters()
            .set("int_param", 100)
            .unwrap();

        assert!(matches!(
            node.declare_parameter("int_param").default(1.0).mandatory(),
            Err(DeclarationError::PriorValueTypeMismatch)
        ));

        let range = ParameterRange {
            lower: Some(-10),
            upper: Some(10),
            step: Some(3),
        };
        assert!(matches!(
            node.declare_parameter("int_param")
                .default(-1)
                .range(range.clone())
                .discriminate(|available| { available.prior_value })
                .mandatory(),
            Err(DeclarationError::InitialValueOutOfRange)
        ));

        {
            // We now ask to discard the mismatching prior value, so we no
            // longer get an error.
            let param = node
                .declare_parameter("int_param")
                .default(1.0)
                .discard_mismatching_prior_value()
                .mandatory()
                .unwrap();
            assert_eq!(param.get(), 1.0);
        }
        {
            // The out of range prior value will be discarded by default.
            node.use_undeclared_parameters()
                .set("int_param", 100)
                .unwrap();
            let param = node
                .declare_parameter("int_param")
                .default(5)
                .range(range)
                .mandatory()
                .unwrap();
            assert_eq!(param.get(), 5);
        }
    }

    #[test]
    fn test_optional_parameter_apis() {
        let ctx = Context::new([]).unwrap();
        let node = create_node(&ctx, &format!("param_test_node_{}", line!())).unwrap();
        node.declare_parameter::<i64>("int_param")
            .optional()
            .unwrap();
    }
}

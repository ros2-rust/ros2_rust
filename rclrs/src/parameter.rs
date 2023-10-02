mod override_map;
mod value;

pub(crate) use override_map::*;
pub use value::*;

use crate::rcl_bindings::*;
use crate::{call_string_getter_with_handle, RclrsError};
use std::collections::BTreeMap;
use std::marker::PhantomData;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc, Mutex, RwLock, Weak,
};

#[derive(Clone, Debug)]
struct ParameterOptionsStorage {
    _description: Arc<str>,
    _constraints: Arc<str>,
    ranges: ParameterRanges,
}

impl<T: ParameterVariant> From<ParameterOptions<T>> for ParameterOptionsStorage {
    fn from(opts: ParameterOptions<T>) -> Self {
        Self {
            _description: opts.description,
            _constraints: opts.constraints,
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

// TODO(luca) we can't derive Default because it requires ParameterVariant to implement Default
impl<T: ParameterVariant> Default for ParameterOptions<T> {
    fn default() -> Self {
        Self {
            description: Arc::from(""),
            constraints: Arc::from(""),
            ranges: Default::default(),
        }
    }
}

impl From<ParameterRange<f64>> for ParameterRanges {
    fn from(params: ParameterRange<f64>) -> Self {
        Self {
            float: Some(params),
            ..Default::default()
        }
    }
}

impl From<ParameterRange<i64>> for ParameterRanges {
    fn from(params: ParameterRange<i64>) -> Self {
        Self {
            integer: Some(params),
            ..Default::default()
        }
    }
}

impl From<()> for ParameterRanges {
    fn from(_empty: ()) -> Self {
        Self::default()
    }
}

/// Contains all the possible type of ranges that can be applied to a value.
/// Usually only one of these ranges will be applied, but all have to be stored since:
///
/// * A dynamic parameter can change its type at runtime, in which case a different range could be
/// applied.
/// * Introspection through service calls requires all the ranges to be reported to the user.
#[derive(Clone, Debug, Default)]
pub struct ParameterRanges {
    float: Option<ParameterRange<f64>>,
    integer: Option<ParameterRange<i64>>,
}

impl ParameterRanges {
    fn validate(&self) -> Result<(), DeclarationError> {
        if let Some(integer) = &self.integer {
            integer.validate()?;
        }
        if let Some(float) = &self.float {
            float.validate()?;
        }
        Ok(())
    }
    fn check_in_range(&self, value: &ParameterValue) -> Result<(), ParameterValueError> {
        match value {
            ParameterValue::Integer(v) => {
                if let Some(range) = &self.integer {
                    range.check(*v)?;
                }
            }
            ParameterValue::Double(v) => {
                if let Some(range) = &self.float {
                    range.check(*v)?;
                }
            }
            _ => {}
        }
        Ok(())
    }
}

/// Describes the range for paramter type T.
#[derive(Clone, Debug, Default)]
pub struct ParameterRange<T: ParameterVariant + PartialOrd> {
    /// Lower limit, if set the parameter must be >= l.
    pub lower: Option<T>,
    /// Upper limit, if set the parameter must be <= u.
    pub upper: Option<T>,
    /// Step size, if set and `lower` is set the parameter must be within an integer number of
    /// steps of size `step` from `lower`, or equal to the upper limit if set.
    /// Example:
    /// If lower is Some(0), upper is Some(10) and step is Some(3), acceptable values are:
    /// [0, 3, 6, 9, 10]
    pub step: Option<T>,
}

impl<T: ParameterVariant + PartialOrd + Default> ParameterRange<T> {
    fn check_boundary(&self, value: &T) -> Result<(), ParameterValueError> {
        if self.lower.as_ref().is_some_and(|l| value < l) {
            return Err(ParameterValueError::OutOfRange);
        }
        if self.upper.as_ref().is_some_and(|u| value > u) {
            return Err(ParameterValueError::OutOfRange);
        }
        Ok(())
    }

    fn validate(&self) -> Result<(), DeclarationError> {
        if self
            .lower
            .as_ref()
            .zip(self.upper.as_ref())
            .is_some_and(|(l, u)| l > u)
        {
            return Err(DeclarationError::InvalidRange);
        }
        if self.step.as_ref().is_some_and(|s| s <= &T::default()) {
            return Err(DeclarationError::InvalidRange);
        }
        Ok(())
    }
}

impl ParameterRange<i64> {
    fn check(&self, value: i64) -> Result<(), ParameterValueError> {
        self.check_boundary(&value)?;
        if self.upper.is_some_and(|u| u == value) {
            return Ok(());
        }
        if let (Some(l), Some(s)) = (self.lower, self.step) {
            if (value - l) % s != 0 {
                return Err(ParameterValueError::OutOfRange);
            }
        }
        Ok(())
    }
}

impl ParameterRange<f64> {
    // Same comparison function as rclcpp.
    fn are_close(v1: f64, v2: f64) -> bool {
        const ULP_TOL: f64 = 100.0;
        (v1 - v2).abs() <= (f64::EPSILON * (v1 + v2).abs() * ULP_TOL)
    }

    fn check(&self, value: f64) -> Result<(), ParameterValueError> {
        if self.upper.is_some_and(|u| Self::are_close(u, value))
            || self.lower.is_some_and(|l| Self::are_close(l, value))
        {
            return Ok(());
        }
        self.check_boundary(&value)?;
        if let (Some(l), Some(s)) = (self.lower, self.step) {
            if !Self::are_close(((value - l) / s).round() * s + l, value) {
                return Err(ParameterValueError::OutOfRange);
            }
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
enum DeclaredValue {
    Mandatory(Arc<RwLock<ParameterValue>>),
    Optional(Arc<RwLock<Option<ParameterValue>>>),
    ReadOnly(ParameterValue),
}

/// Trait used to describe a value that can be declared.
/// Depending on whether a parameter is mandatory or optional, its value can be specified as either
/// `T` or `Option<T>` and this trait assist in defining the types and their constraints.
pub trait Declarable {
    /// Type passed to the builder to specify the default value.
    type DefaultInput;
    /// Type of the parameter.
    type ParameterType: ParameterVariant;
}

impl<T: ParameterVariant> Declarable for T {
    type DefaultInput = T;
    type ParameterType = T;
}

impl<T: ParameterVariant> Declarable for Option<T> {
    type DefaultInput = Option<T>;
    type ParameterType = T;
}

/// Builder used to generate a parameter. Defaults to `ParameterOptions<T>::default()`.
#[must_use]
pub struct ParameterBuilder<'a, T: Declarable> {
    name: String,
    default_value: T,
    tentative: bool,
    options: ParameterOptions<T::ParameterType>,
    interface: &'a ParameterInterface,
}

impl<'a, T: ParameterVariant> ParameterBuilder<'a, Option<T>> {
    // TODO(luca) reduce duplication for setters
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

    /// Sets the parameter's human readable constraints.
    /// These are not enforced by the library but are displayed on parameter description requests
    /// and can be used by integrators to understand complex constraints.
    pub fn tentative(mut self) -> Self {
        self.tentative = true;
        self
    }
    /// Declares the parameter as an Optional parameter, that can be unset.
    ///
    /// Returns:
    /// * `Ok(OptionalParameter<T>)` if declaration was successful.
    /// * `Err(DeclarationError::PreexistingValue(ParameterValueError::OutOfRange))` if the parameter value is out of range.
    /// * `Err(DeclarationError::Override(ParameterValueError::OutOfRange))` if the parameter override is out of range.
    /// * `Err(DeclarationError::Override(ParameterValueError::TypeMismatch))` if the parameter override is set to the wrong type.
    pub fn optional(self) -> Result<OptionalParameter<T>, DeclarationError> {
        self.interface.declare_optional(
            &self.name,
            self.default_value,
            self.options,
            self.tentative,
        )
    }
}

impl<'a, T: ParameterVariant> ParameterBuilder<'a, T> {
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

    /// Sets the parameter's human readable constraints.
    /// These are not enforced by the library but are displayed on parameter description requests
    /// and can be used by integrators to understand complex constraints.
    pub fn tentative(mut self) -> Self {
        self.tentative = true;
        self
    }

    /// Declares the parameter as a Mandatory parameter, that must always have a value set.
    ///
    /// Returns:
    /// * `Ok(MandatoryParameter<T>)` if declaration was successful.
    /// * `Err(DeclarationError::AlreadyDeclared)` if the parameter was already declared.
    /// * `Err(DeclarationError::PreexistingValue(ParameterValueError::OutOfRange))` if the parameter value is out of range.
    /// * `Err(DeclarationError::Override(ParameterValueError::OutOfRange))` if the parameter override is out of range.
    /// * `Err(DeclarationError::Override(ParameterValueError::TypeMismatch))` if the parameter override is set to the wrong type.
    pub fn mandatory(self) -> Result<MandatoryParameter<T>, DeclarationError> {
        self.try_into()
    }

    /// Declares the parameter as a ReadOnly parameter, that cannot be edited.
    ///
    /// Returns:
    /// * `Ok(ReadOnlyParameter<T>)` if declaration was successful.
    /// * `Err(DeclarationError::PreexistingValue(ParameterValueError::OutOfRange))` if the parameter value is out of range.
    /// * `Err(DeclarationError::Override(ParameterValueError::OutOfRange))` if the parameter override is out of range.
    /// * `Err(DeclarationError::Override(ParameterValueError::TypeMismatch))` if the parameter override is set to the wrong type.
    pub fn read_only(self) -> Result<ReadOnlyParameter<T>, DeclarationError> {
        self.try_into()
    }

    /// Declares the parameter as an Optional parameter, that can be unset.
    ///
    /// Returns:
    /// * `Ok(OptionalParameter<T>)` if declaration was successful.
    /// * `Err(DeclarationError::PreexistingValue(ParameterValueError::OutOfRange))` if the parameter value is out of range.
    /// * `Err(DeclarationError::Override(ParameterValueError::OutOfRange))` if the parameter override is out of range.
    /// * `Err(DeclarationError::Override(ParameterValueError::TypeMismatch))` if the parameter override is set to the wrong type.
    pub fn optional(self) -> Result<OptionalParameter<T>, DeclarationError> {
        self.interface.declare_optional(
            &self.name,
            Some(self.default_value),
            self.options,
            self.tentative,
        )
    }
}

/// A parameter that must have a value
/// This struct has ownership of the declared parameter. Additional parameter declaration will fail
/// while this struct exists and the parameter will be undeclared when it is dropped.
pub struct MandatoryParameter<T: ParameterVariant> {
    name: String,
    value: Arc<RwLock<ParameterValue>>,
    ranges: ParameterRanges,
    map: Weak<Mutex<ParameterMap>>,
    _marker: PhantomData<T>,
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

impl<T: ParameterVariant> TryFrom<ParameterBuilder<'_, T>> for MandatoryParameter<T> {
    type Error = DeclarationError;

    fn try_from(builder: ParameterBuilder<T>) -> Result<Self, Self::Error> {
        let ranges = builder.options.ranges.clone().into();
        let default_value = builder.default_value.into();
        ParameterInterface::validate_range_and_default_value(&ranges, Some(&default_value))?;
        let value = builder
            .interface
            .get_declaration_default_value::<T>(&builder.name, &ranges, builder.tentative)?
            .unwrap_or(default_value);
        let value = Arc::new(RwLock::new(value));
        builder.interface.store_parameter(
            &builder.name,
            T::kind(),
            DeclaredValue::Mandatory(value.clone()),
            builder.options.into(),
        );
        Ok(MandatoryParameter {
            name: builder.name,
            value,
            ranges,
            map: Arc::downgrade(&builder.interface._parameter_map),
            _marker: Default::default(),
        })
    }
}

/// A parameter that might not have a value, represented by `Option<T>`.
/// This struct has ownership of the declared parameter. Additional parameter declaration will fail
/// while this struct exists and the parameter will be undeclared when it is dropped.
pub struct OptionalParameter<T: ParameterVariant> {
    name: String,
    value: Arc<RwLock<Option<ParameterValue>>>,
    ranges: ParameterRanges,
    map: Weak<Mutex<ParameterMap>>,
    _marker: PhantomData<T>,
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
    name: String,
    value: ParameterValue,
    map: Weak<Mutex<ParameterMap>>,
    _marker: PhantomData<T>,
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

impl<T: ParameterVariant> TryFrom<ParameterBuilder<'_, T>> for ReadOnlyParameter<T> {
    type Error = DeclarationError;

    fn try_from(builder: ParameterBuilder<T>) -> Result<Self, Self::Error> {
        let ranges = builder.options.ranges.clone().into();
        let default_value = builder.default_value.into();
        ParameterInterface::validate_range_and_default_value(&ranges, Some(&default_value))?;
        let value = builder
            .interface
            .get_declaration_default_value::<T>(&builder.name, &ranges, builder.tentative)?
            .unwrap_or(default_value);
        builder.interface.store_parameter(
            &builder.name,
            T::kind(),
            DeclaredValue::ReadOnly(value.clone()),
            builder.options.into(),
        );
        Ok(ReadOnlyParameter {
            name: builder.name,
            value,
            map: Arc::downgrade(&builder.interface._parameter_map),
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
    Undeclared(Arc<RwLock<ParameterValue>>),
}

#[derive(Debug, Default)]
struct ParameterMap {
    storage: BTreeMap<String, ParameterStorage>,
}

impl<T: ParameterVariant> MandatoryParameter<T> {
    /// Returns a clone of the most recent value of the parameter.
    pub fn get(&self) -> T {
        T::maybe_from(self.value.read().unwrap().clone()).unwrap()
    }

    /// Sets the parameter value.
    /// Returns `DeclarationError::OutOfRange` if the value is out of the parameter's range.
    pub fn set<U: Into<T>>(&self, value: U) -> Result<(), ParameterValueError> {
        let value = value.into().into();
        self.ranges.check_in_range(&value)?;
        *self.value.write().unwrap() = value;
        Ok(())
    }
}

impl<T: ParameterVariant> ReadOnlyParameter<T> {
    /// Returns a clone of the most recent value of the parameter.
    pub fn get(&self) -> T {
        T::maybe_from(self.value.clone()).unwrap()
    }
}

impl<T: ParameterVariant> OptionalParameter<T> {
    /// Returns a clone of the most recent value of the parameter.
    pub fn get(&self) -> Option<T> {
        self.value
            .read()
            .unwrap()
            .clone()
            .map(|p| T::maybe_from(p).unwrap())
    }

    /// Assigns a value to the optional parameter, setting it to `Some(value)`.
    /// Returns `DeclarationError::OutOfRange` if the value is out of the parameter's range.
    pub fn set<U: Into<T>>(&self, value: U) -> Result<(), ParameterValueError> {
        let value = value.into().into();
        self.ranges.check_in_range(&value)?;
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

/// Error that can be generated when doing operations on parameters.
#[derive(Debug)]
pub enum DeclarationError {
    /// Parameter was already declared and a new declaration was attempted.
    AlreadyDeclared,
    /// An error caused when trying to set the parameter's value.
    PreexistingValue(ParameterValueError),
    /// An error caused when parsing the parameter override's value.
    Override(ParameterValueError),
    /// An invalid range was provided to a parameter declaration (i.e. lower bound > higher bound).
    InvalidRange,
    /// The default value provided to the declaration was out of range.
    DefaultOutOfRange,
}

impl<'a> Parameters<'a> {
    /// Tries to read a parameter of the requested type.
    ///
    /// Returns Some(T) if a parameter of the requested type exists, None otherwise.
    pub fn get<T: ParameterVariant>(&self, name: &str) -> Option<T> {
        let storage = &self.interface._parameter_map.lock().unwrap().storage;
        let Some(storage) = storage.get(name) else {
            return None;
        };
        match storage {
            ParameterStorage::Declared(storage) => match &storage.value {
                DeclaredValue::Mandatory(p) => T::maybe_from(p.read().unwrap().clone()),
                DeclaredValue::Optional(p) => {
                    p.read().unwrap().clone().and_then(|p| T::maybe_from(p))
                }
                DeclaredValue::ReadOnly(p) => T::maybe_from(p.clone()),
            },
            ParameterStorage::Undeclared(value) => T::maybe_from(value.read().unwrap().clone()),
        }
    }

    /// Tries to set a parameter with the requested value.
    ///
    /// Returns:
    /// * Ok(()) if setting was successful.
    /// * Err(DeclarationError::TypeMismatch) if the type of the requested value is different from
    /// the parameter's type.
    pub fn set<T: ParameterVariant>(
        &self,
        name: &str,
        value: T,
    ) -> Result<(), ParameterValueError> {
        let mut map = self.interface._parameter_map.lock().unwrap();
        if let Some(entry) = map.storage.get_mut(name) {
            // If it's declared we can only set if it's the same variant.
            // Undeclared parameters are dynamic by default
            match entry {
                ParameterStorage::Declared(param) => {
                    if T::kind() == param.kind {
                        let value = value.into();
                        param.options.ranges.check_in_range(&value)?;
                        match &param.value {
                            DeclaredValue::Mandatory(p) => *p.write().unwrap() = value,
                            DeclaredValue::Optional(p) => *p.write().unwrap() = Some(value),
                            DeclaredValue::ReadOnly(_) => {
                                return Err(ParameterValueError::ReadOnly)
                            }
                        }
                    } else {
                        return Err(ParameterValueError::TypeMismatch);
                    }
                }
                ParameterStorage::Undeclared(param) => {
                    *param.write().unwrap() = value.into();
                }
            }
        } else {
            map.storage.insert(
                name.to_owned(),
                ParameterStorage::Undeclared(Arc::new(RwLock::new(value.into()))),
            );
        }
        Ok(())
    }
}

pub(crate) struct ParameterInterface {
    _parameter_map: Arc<Mutex<ParameterMap>>,
    _override_map: ParameterOverrideMap,
    allow_undeclared: AtomicBool,
    //_services: ParameterService,
}

impl ParameterInterface {
    pub(crate) fn new(
        rcl_node_mtx: &Arc<Mutex<rcl_node_t>>,
        node_arguments: &rcl_arguments_t,
        global_arguments: &rcl_arguments_t,
    ) -> Result<Self, RclrsError> {
        //let _services = ParameterService::new(rcl_node_mtx)?;

        let rcl_node = rcl_node_mtx.lock().unwrap();
        let _override_map = unsafe {
            let fqn = call_string_getter_with_handle(&rcl_node, rcl_node_get_fully_qualified_name);
            resolve_parameter_overrides(&fqn, node_arguments, global_arguments)?
        };

        Ok(ParameterInterface {
            _parameter_map: Default::default(),
            _override_map,
            allow_undeclared: Default::default(),
            //_services,
        })
    }

    pub(crate) fn declare<T: Declarable>(
        &self,
        name: &str,
        default_value: T,
    ) -> ParameterBuilder<T> {
        ParameterBuilder {
            default_value,
            name: name.into(),
            tentative: false,
            options: Default::default(),
            interface: self,
        }
    }

    fn get_declaration_default_value<T: ParameterVariant>(
        &self,
        name: &str,
        ranges: &ParameterRanges,
        tentative: bool,
    ) -> Result<Option<ParameterValue>, DeclarationError> {
        let mut value = None;
        if let Some(param_override) = self._override_map.get(name) {
            ranges
                .check_in_range(param_override)
                .map_err(DeclarationError::Override)?;
            value = Some(T::maybe_from(param_override.clone()).ok_or(
                DeclarationError::Override(ParameterValueError::TypeMismatch),
            )?);
        }
        if let Some(current_value) = self._parameter_map.lock().unwrap().storage.get(name) {
            match current_value {
                ParameterStorage::Declared(_) => return Err(DeclarationError::AlreadyDeclared),
                ParameterStorage::Undeclared(param) => {
                    let param = param.read().unwrap().clone();
                    if let Err(e) = ranges
                        .check_in_range(&param)
                        .map_err(DeclarationError::PreexistingValue)
                    {
                        if tentative {
                            return Err(e);
                        } else {
                            return Ok(value.map(|v| v.into()));
                        }
                    }
                    if let Some(v) = T::maybe_from(param) {
                        value = Some(v);
                    } else if tentative {
                        return Err(DeclarationError::PreexistingValue(
                            ParameterValueError::TypeMismatch,
                        ));
                    }
                }
            }
        }
        Ok(value.map(|v| v.into()))
    }

    pub(crate) fn declare_from_iter<U: IntoIterator>(
        &self,
        name: &str,
        default_value: U,
    ) -> ParameterBuilder<Arc<[U::Item]>>
    where
        Arc<[U::Item]>: ParameterVariant,
    {
        // TODO(luca) consider passing a FnOnce to initialize the value to declare to do lazy
        // initialization.
        let value = default_value.into_iter().collect();
        self.declare(name, value)
    }

    pub(crate) fn declare_string_array<U>(
        &self,
        name: &str,
        default_value: U,
    ) -> ParameterBuilder<Arc<[Arc<str>]>>
    where
        U: IntoIterator,
        U::Item: Into<Arc<str>>,
    {
        // TODO(luca) consider passing a FnOnce to initialize the value to declare to do lazy
        // initialization.
        let value = default_value.into_iter().map(|v| v.into()).collect();
        self.declare(name, value)
    }

    fn declare_optional<T: ParameterVariant>(
        &self,
        name: &str,
        default_value: Option<T>,
        options: ParameterOptions<T>,
        tentative: bool,
    ) -> Result<OptionalParameter<T>, DeclarationError> {
        let ranges = options.ranges.clone().into();
        let default_value = default_value.map(|v| v.into());
        Self::validate_range_and_default_value(&ranges, default_value.as_ref())?;
        let value = self
            .get_declaration_default_value::<T>(name, &ranges, tentative)?
            .or(default_value);
        let value = Arc::new(RwLock::new(value));
        self.store_parameter(
            name,
            T::kind(),
            DeclaredValue::Optional(value.clone()),
            options.into(),
        );
        Ok(OptionalParameter {
            name: name.to_owned(),
            value,
            ranges,
            map: Arc::downgrade(&self._parameter_map),
            _marker: Default::default(),
        })
    }

    fn validate_range_and_default_value(
        ranges: &ParameterRanges,
        value: Option<&ParameterValue>,
    ) -> Result<(), DeclarationError> {
        ranges.validate()?;
        if let Some(v) = value {
            ranges
                .check_in_range(v)
                .map_err(|_| DeclarationError::DefaultOutOfRange)?;
        }
        Ok(())
    }

    fn store_parameter(
        &self,
        name: &str,
        kind: ParameterKind,
        value: DeclaredValue,
        options: ParameterOptionsStorage,
    ) {
        self._parameter_map.lock().unwrap().storage.insert(
            name.to_owned(),
            ParameterStorage::Declared(DeclaredStorage {
                options,
                value,
                kind,
            }),
        );
    }

    pub(crate) fn allow_undeclared(&self) {
        self.allow_undeclared.store(true, Ordering::Relaxed);
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
        let node = create_node(&ctx, "param_test_node").unwrap();

        // Declaring a parameter with a different type than what was overridden should return an
        // error
        assert!(matches!(
            node.declare_parameter("declared_int", 1.0).mandatory(),
            Err(DeclarationError::Override(
                ParameterValueError::TypeMismatch
            ))
        ));

        // If the override does not respect the range we should return an error
        let range = ParameterRange {
            upper: Some(5),
            ..Default::default()
        };
        assert!(matches!(
            node.declare_parameter("declared_int", 1)
                .range(range)
                .mandatory(),
            Err(DeclarationError::Override(ParameterValueError::OutOfRange))
        ));
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
        let node = create_node(&ctx, "param_test_node").unwrap();

        let overridden_int = node
            .declare_parameter("declared_int", 123)
            .mandatory()
            .unwrap();
        assert_eq!(overridden_int.get(), 10);

        let new_param = node
            .declare_parameter("new_param", 2.0)
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
                .declare_parameter("new_bool", false)
                .mandatory()
                .unwrap();
            assert!(bool_param.get());
        }
        {
            node.use_undeclared_parameters()
                .set("new_bool", true)
                .unwrap();
            let bool_param = node
                .declare_parameter("new_bool", false)
                .optional()
                .unwrap();
            assert_eq!(bool_param.get(), Some(true));
        }

        let optional_param = node
            .declare_parameter("non_existing_bool", None)
            .optional()
            .unwrap();
        assert_eq!(optional_param.get(), None);
        optional_param.set(true).unwrap();
        assert_eq!(optional_param.get(), Some(true));
        optional_param.unset();
        assert_eq!(optional_param.get(), None);

        let optional_param2 = node
            .declare_parameter("non_existing_bool2", Some(false))
            .optional()
            .unwrap();
        assert_eq!(optional_param2.get(), Some(false));

        // This was provided as a parameter override, hence should be set to true
        let optional_param3 = node
            .declare_parameter("optional_bool", Some(false))
            .optional()
            .unwrap();
        assert_eq!(optional_param3.get(), Some(true));

        // double_array was overriden to 1.0 2.0 through command line overrides
        let array_param = node
            .declare_parameter_from_iter("double_array", vec![10.0, 20.0])
            .mandatory()
            .unwrap();
        assert_eq!(array_param.get()[0], 1.0);
        assert_eq!(array_param.get()[1], 2.0);

        let array_param = node
            .declare_string_array_parameter("string_array", vec!["Hello", "World"])
            .mandatory()
            .unwrap();
        assert_eq!(array_param.get()[0], "Hello".into());
        assert_eq!(array_param.get()[1], "World".into());

        // If a value is set when undeclared, a following declare_parameter should have the
        // previously set value.
        node.use_undeclared_parameters()
            .set("undeclared_int", 42)
            .unwrap();
        let undeclared_int = node
            .declare_parameter("undeclared_int", 10)
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
        let node = create_node(&ctx, "param_test_node").unwrap();
        // If a parameter was set as an override and as an undeclared parameter, the undeclared
        // value should get priority
        node.use_undeclared_parameters()
            .set("declared_int", 20)
            .unwrap();
        let param = node
            .declare_parameter("declared_int", 30)
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
        let node = create_node(&ctx, "param_test_node").unwrap();
        {
            // Setting a parameter with an override
            let param = node
                .declare_parameter("declared_int", 1)
                .mandatory()
                .unwrap();
            assert_eq!(param.get(), 10);
            param.set(2).unwrap();
            assert_eq!(param.get(), 2);
            // Redeclaring should fail
            assert!(matches!(
                node.declare_parameter("declared_int", 1).mandatory(),
                Err(DeclarationError::AlreadyDeclared)
            ));
        }
        {
            // Parameter went out of scope, redeclaring should be OK and return command line
            // override
            let param = node
                .declare_parameter("declared_int", 1)
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
        let node = create_node(&ctx, "param_test_node").unwrap();
        // Setting invalid ranges should fail
        let range = ParameterRange {
            lower: Some(10),
            upper: Some(-10),
            step: Some(3),
        };
        assert!(matches!(
            node.declare_parameter("int_param", 5)
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
            node.declare_parameter("int_param", 5)
                .range(range)
                .mandatory(),
            Err(DeclarationError::InvalidRange)
        ));
        // Setting parameters out of range range should fail
        let range = ParameterRange {
            lower: Some(-10),
            upper: Some(10),
            step: Some(3),
        };
        assert!(matches!(
            node.declare_parameter("out_of_range_int", 100)
                .range(range.clone())
                .mandatory(),
            Err(DeclarationError::DefaultOutOfRange)
        ));
        assert!(matches!(
            node.declare_parameter("wrong_step_int", -9)
                .range(range.clone())
                .mandatory(),
            Err(DeclarationError::DefaultOutOfRange)
        ));
        let param = node
            .declare_parameter("int_param", -7)
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
            node.declare_parameter("out_of_range_double", 100.0)
                .range(range.clone())
                .optional(),
            Err(DeclarationError::DefaultOutOfRange)
        ));
        assert!(matches!(
            node.declare_parameter("wrong_step_double", -9.0)
                .range(range.clone())
                .read_only(),
            Err(DeclarationError::DefaultOutOfRange)
        ));
        let param = node
            .declare_parameter("double_param", -7.0)
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
        let node = create_node(&ctx, "param_test_node").unwrap();
        let param = node
            .declare_parameter("int_param", 100)
            .read_only()
            .unwrap();
        // Multiple copies cannot be declared
        assert!(matches!(
            node.declare_parameter("int_param", 100).read_only(),
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
    fn test_preexisting_value_error_and_tentative_api() {
        let ctx = Context::new([]).unwrap();
        let node = create_node(&ctx, "param_test_node").unwrap();
        node.use_undeclared_parameters()
            .set("int_param", 100)
            .unwrap();
        // Tentative will try to use previously set values and return errors if they are out of
        // range or of the wrong type
        assert!(matches!(
            node.declare_parameter("int_param", 1.0)
                .tentative()
                .mandatory(),
            Err(DeclarationError::PreexistingValue(
                ParameterValueError::TypeMismatch
            ))
        ));

        let range = ParameterRange {
            lower: Some(-10),
            upper: Some(10),
            step: Some(3),
        };
        assert!(matches!(
            node.declare_parameter("int_param", -1)
                .range(range.clone())
                .tentative()
                .mandatory(),
            Err(DeclarationError::PreexistingValue(
                ParameterValueError::OutOfRange
            ))
        ));

        // Default behavior will just drop the previously set value if it is out of range or of the
        // wrong type.
        {
            let param = node
                .declare_parameter("int_param", 1.0)
                .mandatory()
                .unwrap();
            assert_eq!(param.get(), 1.0);
        }
        {
            node.use_undeclared_parameters()
                .set("int_param", 100)
                .unwrap();
            let param = node
                .declare_parameter("int_param", 5)
                .range(range)
                .mandatory()
                .unwrap();
            assert_eq!(param.get(), 5);
        }
    }

    #[test]
    fn test_optional_parameter_apis() {
        let ctx = Context::new([]).unwrap();
        let node = create_node(&ctx, "param_test_node").unwrap();
        let range = ParameterRange {
            lower: Some(-10),
            upper: Some(10),
            step: None,
        };
        {
            node.declare_parameter("int_param", 1).optional().unwrap();
        }
        {
            node.declare_parameter("int_param", Some(1))
                .range(range)
                .optional()
                .unwrap();
        }
        {
            node.declare_unset_parameter::<i64>("int_param")
                .optional()
                .unwrap();
        }
    }
}

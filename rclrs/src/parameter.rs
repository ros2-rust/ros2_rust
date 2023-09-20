mod override_map;
mod service;
mod value;

pub(crate) use override_map::*;
pub(crate) use service::*;
pub use value::*;

use crate::rcl_bindings::*;
use crate::{call_string_getter_with_handle, RclrsError};
use std::collections::{BTreeMap, HashMap};
use std::marker::PhantomData;
use std::sync::{Arc, Mutex, RwLock};

// TODO(luca) add From for rcl_interfaces::ParameterDescriptor message, but a manual implementation
// for the opposite since this struct does not contain all the fields required to populate the
// message
#[derive(Clone, Debug)]
pub struct ParameterOptions {
    // TODO(luca) add int / float range
}

#[derive(Clone, Debug)]
enum DeclaredValue {
    Mandatory(Arc<RwLock<ParameterValue>>),
    Optional(Arc<RwLock<Option<ParameterValue>>>),
}

pub struct MandatoryParameter<T: ParameterVariant> {
    value: Arc<RwLock<ParameterValue>>,
    _marker: PhantomData<T>,
}

pub struct OptionalParameter<T: ParameterVariant> {
    value: Arc<RwLock<Option<ParameterValue>>>,
    _marker: PhantomData<T>,
}

#[derive(Clone, Debug)]
struct DeclaredStorage {
    value: DeclaredValue,
    kind: ParameterKind,
    options: ParameterOptions,
}

#[derive(Clone, Debug, Default)]
struct ParameterStorage {
    declared: HashMap<String, DeclaredStorage>,
    undeclared: Option<HashMap<String, ParameterValue>>,
}

impl<T: ParameterVariant> MandatoryParameter<T> {
    pub fn get(&self) -> T {
        T::maybe_from(self.value.read().unwrap().clone()).unwrap()
    }

    pub fn set(&self, value: T) {
        *self.value.write().unwrap() = value.into();
    }
}

impl<T: ParameterVariant> OptionalParameter<T> {
    pub fn get(&self) -> Option<T> {
        self.value
            .read()
            .unwrap()
            .clone()
            .map(|p| T::maybe_from(p).unwrap())
    }

    pub fn set(&self, value: T) {
        *self.value.write().unwrap() = Some(value.into());
    }
}

pub struct ParameterInterface {
    _parameter_storage: ParameterStorage,
    _override_map: ParameterOverrideMap,
    _services: ParameterService,
}

impl ParameterInterface {
    pub(crate) fn new(
        rcl_node_mtx: &Arc<Mutex<rcl_node_t>>,
        node_arguments: &rcl_arguments_t,
        global_arguments: &rcl_arguments_t,
    ) -> Result<Self, RclrsError> {
        let _services = ParameterService::new(rcl_node_mtx)?;

        let rcl_node = rcl_node_mtx.lock().unwrap();
        let _override_map = unsafe {
            let fqn = call_string_getter_with_handle(&rcl_node, rcl_node_get_fully_qualified_name);
            resolve_parameter_overrides(&fqn, node_arguments, global_arguments)?
        };

        Ok(ParameterInterface {
            _parameter_storage: Default::default(),
            _override_map,
            _services,
        })
    }

    pub fn declare<T: ParameterVariant>(
        &mut self,
        name: &str,
        options: ParameterOptions,
        default_value: T,
    ) -> MandatoryParameter<T> {
        let mut value = default_value.into();
        if let Some(param_override) = self._override_map.get(name) {
            // TODO(luca) It is possible for the override (i.e. through command line) to be of a
            // different type thant what is declared, in which case we ignore the override.
            // We currently print an error but there should probably be more formal error
            // reporting.
            if param_override.static_kind() == T::kind() {
                value = param_override.clone();
            } else {
                println!("Mismatch in parameter override type for {}, ignoring", name);
            }
        }
        let value = Arc::new(RwLock::new(value));
        self._parameter_storage.declared.insert(
            name.to_owned(),
            DeclaredStorage {
                options,
                value: DeclaredValue::Mandatory(value.clone()),
                kind: T::kind(),
            },
        );
        MandatoryParameter {
            value,
            _marker: Default::default(),
        }
    }

    pub fn declare_optional<T: ParameterVariant>(
        &mut self,
        name: &str,
        options: ParameterOptions,
        default_value: Option<T>,
    ) -> OptionalParameter<T> {
        let mut value = default_value.map(|p| p.into());
        if let Some(param_override) = self._override_map.get(name) {
            // TODO(luca) It is possible for the override (i.e. through command line) to be of a
            // different type thant what is declared, in which case we ignore the override.
            // We currently print an error but there should probably be more formal error
            // reporting.
            if param_override.static_kind() == T::kind() {
                value = Some(param_override.clone());
            } else {
                println!("Mismatch in parameter override type for {}, ignoring", name);
            }
        }
        let value = Arc::new(RwLock::new(value));
        self._parameter_storage.declared.insert(
            name.to_owned(),
            DeclaredStorage {
                options,
                value: DeclaredValue::Optional(value.clone()),
                kind: T::kind(),
            },
        );
        OptionalParameter {
            value,
            _marker: Default::default(),
        }
    }

    pub fn get<T: ParameterVariant>(&self, name: &str) -> Option<T> {
        let Some(storage) = self._parameter_storage.declared.get(name) else {
            return None;
        };
        match &storage.value {
            DeclaredValue::Mandatory(v) => Some(T::maybe_from(v.read().unwrap().clone()).unwrap()),
            DeclaredValue::Optional(v) => {
                v.read().unwrap().clone().map(|p| T::maybe_from(p).unwrap())
            }
        }
    }

    // TODO(luca) either implement a new error or a new RclrsError variant
    pub fn set<T: ParameterVariant>(&mut self, name: &str, value: T) -> Result<(), ()> {
        match self._parameter_storage.declared.get_mut(name) {
            Some(storage) => {
                if T::kind() == storage.kind {
                    // TODO(luca) design document requires ability to "try" setting parameters
                    // without actually doing it, this will be especially needed for setting
                    // parameters atomically
                    match &storage.value {
                        DeclaredValue::Mandatory(v) => *v.write().unwrap() = value.into(),
                        DeclaredValue::Optional(v) => *v.write().unwrap() = Some(value.into()),
                    }
                    Ok(())
                } else {
                    // Trying to set parameter of the wrong type and parameter is not dynamic
                    Err(())
                }
            }
            // Parameter was not declared
            None => Err(()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Context, RclrsError, ToResult};

    #[test]
    fn test_parameter_setting_declaring() {}
}

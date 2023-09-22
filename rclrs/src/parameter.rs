mod override_map;
mod service;
mod value;

pub(crate) use override_map::*;
pub(crate) use service::*;
pub use value::*;

use crate::rcl_bindings::*;
use crate::{call_string_getter_with_handle, RclrsError};
use std::collections::{hash_map::Entry, BTreeMap, HashMap};
use std::marker::PhantomData;
use std::sync::{Arc, Mutex, RwLock, Weak};

// TODO(luca) add From for rcl_interfaces::ParameterDescriptor message, but a manual implementation
// for the opposite since this struct does not contain all the fields required to populate the
// message
#[derive(Clone, Debug, Default)]
pub struct ParameterOptions {
    // TODO(luca) add int / float range
}

// We use weak references since parameters are owned by the declarer, not the storage.
#[derive(Clone, Debug)]
enum DeclaredValue {
    Mandatory(Arc<RwLock<ParameterValue>>),
    Optional(Arc<RwLock<Option<ParameterValue>>>),
}

pub struct MandatoryParameter<T: ParameterVariant> {
    name: String,
    value: Arc<RwLock<ParameterValue>>,
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

pub struct OptionalParameter<T: ParameterVariant> {
    name: String,
    value: Arc<RwLock<Option<ParameterValue>>>,
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

#[derive(Clone, Debug)]
struct DeclaredStorage {
    value: DeclaredValue,
    kind: ParameterKind,
    options: ParameterOptions,
}

#[derive(Debug)]
enum ParameterStorage {
    Declared(DeclaredStorage),
    Undeclared(Arc<RwLock<ParameterValue>>),
}

#[derive(Debug, Default)]
struct ParameterMap {
    storage: HashMap<String, ParameterStorage>,
    allow_undeclared: bool,
}

impl<T: ParameterVariant> MandatoryParameter<T> {
    pub fn get(&self) -> T {
        T::maybe_from(self.value.read().unwrap().clone()).unwrap()
    }

    pub fn set<U: Into<T>>(&self, value: U) {
        *self.value.write().unwrap() = value.into().into();
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

pub struct AllParameters {
    _parameter_map: Arc<Mutex<ParameterMap>>,
}

impl AllParameters {
    pub(crate) fn new(parameter_interface: &ParameterInterface) -> Self {
        parameter_interface.allow_undeclared();
        Self {
            _parameter_map: parameter_interface._parameter_map.clone(),
        }
    }

    pub fn get<T: ParameterVariant>(&self, name: &str) -> Option<T> {
        let storage = &self._parameter_map.lock().unwrap().storage;
        let Some(storage) = storage.get(name) else {
            return None;
        };
        match storage {
            ParameterStorage::Declared(storage) => match &storage.value {
                DeclaredValue::Mandatory(p) => T::maybe_from(p.read().unwrap().clone()),
                DeclaredValue::Optional(p) => {
                    p.read().unwrap().clone().and_then(|p| T::maybe_from(p))
                }
            },
            ParameterStorage::Undeclared(value) => T::maybe_from(value.read().unwrap().clone()),
        }
    }

    // TODO(luca) either implement a new error or a new RclrsError variant
    pub fn set<T: ParameterVariant>(&self, name: &str, value: T) -> Result<(), ()> {
        match self
            ._parameter_map
            .lock()
            .unwrap()
            .storage
            .entry(name.to_string())
        {
            Entry::Occupied(mut entry) => {
                // If it's declared we can only set if it's the same variant.
                // Undeclared parameters are dynamic by default
                match entry.get_mut() {
                    ParameterStorage::Declared(param) => {
                        if T::kind() == param.kind {
                            match &param.value {
                                DeclaredValue::Mandatory(p) => *p.write().unwrap() = value.into(),
                                DeclaredValue::Optional(p) => {
                                    *p.write().unwrap() = Some(value.into())
                                }
                            }
                            Ok(())
                        } else {
                            Err(())
                        }
                    }
                    ParameterStorage::Undeclared(param) => {
                        *param.write().unwrap() = value.into();
                        Ok(())
                    }
                }
            }
            Entry::Vacant(entry) => {
                entry.insert(ParameterStorage::Undeclared(Arc::new(RwLock::new(
                    value.into(),
                ))));
                Ok(())
            }
        }
    }
}

pub(crate) struct ParameterInterface {
    _parameter_map: Arc<Mutex<ParameterMap>>,
    _override_map: ParameterOverrideMap,
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
            //_services,
        })
    }

    pub(crate) fn declare<T: ParameterVariant>(
        &self,
        name: &str,
        default_value: T,
        options: ParameterOptions,
    ) -> Result<MandatoryParameter<T>, ()> {
        let mut value = default_value.into();
        if let Some(current_value) = self._parameter_map.lock().unwrap().storage.get(name) {
            match current_value {
                ParameterStorage::Declared(_) => return Err(()),
                ParameterStorage::Undeclared(param) => {
                    if let Some(v) = T::maybe_from(param.read().unwrap().clone()) {
                        value = v.into();
                    }
                }
            }
        }
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
        self._parameter_map.lock().unwrap().storage.insert(
            name.to_owned(),
            ParameterStorage::Declared(DeclaredStorage {
                options,
                value: DeclaredValue::Mandatory(value.clone()),
                kind: T::kind(),
            }),
        );
        Ok(MandatoryParameter {
            name: name.to_owned(),
            value,
            map: Arc::downgrade(&self._parameter_map),
            _marker: Default::default(),
        })
    }

    pub(crate) fn declare_optional<T: ParameterVariant>(
        &self,
        name: &str,
        default_value: Option<T>,
        options: ParameterOptions,
    ) -> Result<OptionalParameter<T>, ()> {
        let mut value = default_value.map(|p| p.into());
        // TODO(luca) refactor duplicated code between mandatory and optional declaration
        if let Some(current_value) = self._parameter_map.lock().unwrap().storage.get(name) {
            match current_value {
                ParameterStorage::Declared(_) => return Err(()),
                ParameterStorage::Undeclared(param) => {
                    if let Some(v) = T::maybe_from(param.read().unwrap().clone()) {
                        value = Some(v.into());
                    }
                }
            }
        }
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
        self._parameter_map.lock().unwrap().storage.insert(
            name.to_owned(),
            ParameterStorage::Declared(DeclaredStorage {
                options,
                value: DeclaredValue::Optional(value.clone()),
                kind: T::kind(),
            }),
        );
        Ok(OptionalParameter {
            name: name.to_owned(),
            value,
            map: Arc::downgrade(&self._parameter_map),
            _marker: Default::default(),
        })
    }

    fn allow_undeclared(&self) {
        self._parameter_map.lock().unwrap().allow_undeclared = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{create_node, Context, RclrsError};

    #[test]
    fn test_parameter_setting_declaring() -> Result<(), RclrsError> {
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
        ])?;
        let node = create_node(&ctx, "param_test_node")?;

        let overridden_int = node
            .declare_parameter("declared_int", 123, ParameterOptions::default())
            .unwrap();
        assert_eq!(overridden_int.get(), 10);

        let new_param = node
            .declare_parameter("new_param", 2.0, ParameterOptions::default())
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
        assert!(node
            .use_undeclared_parameters()
            .set("new_param", 42)
            .is_err());

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
        new_param.set(5.0);
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

        let optional_param = node
            .declare_optional_parameter::<bool>(
                "non_existing_bool",
                None,
                ParameterOptions::default(),
            )
            .unwrap();
        assert_eq!(optional_param.get(), None);

        let optional_param2 = node
            .declare_optional_parameter(
                "non_existing_bool2",
                Some(false),
                ParameterOptions::default(),
            )
            .unwrap();
        assert_eq!(optional_param2.get(), Some(false));

        // This was provided as a parameter override, hence should be set to true
        let optional_param3 = node
            .declare_optional_parameter("optional_bool", Some(false), ParameterOptions::default())
            .unwrap();
        assert_eq!(optional_param3.get(), Some(true));

        // Test syntax for array types
        let double_array = node
            .declare_parameter::<Arc<[f64]>>(
                "double_array",
                vec![10.0, 20.0].into(),
                ParameterOptions::default(),
            )
            .unwrap();

        // TODO(luca) clearly UX for array types can be improved
        let strings = Arc::from([Arc::from("Hello"), Arc::from("World")]);
        let string_array = node
            .declare_parameter::<Arc<[Arc<str>]>>(
                "string_array",
                strings,
                ParameterOptions::default(),
            )
            .unwrap();

        // If a value is set when undeclared, a following declare_parameter should have the
        // previously set value.
        node.use_undeclared_parameters().set("undeclared_int", 42);
        let undeclared_int = node
            .declare_parameter("undeclared_int", 10, ParameterOptions::default())
            .unwrap();
        assert_eq!(undeclared_int.get(), 42);

        Ok(())
    }

    #[test]
    fn test_parameter_scope_redeclaring() -> Result<(), RclrsError> {
        let ctx = Context::new([
            String::from("--ros-args"),
            String::from("-p"),
            String::from("declared_int:=10"),
        ])?;
        let node = create_node(&ctx, "param_test_node")?;
        {
            // Setting a parameter with an override
            let param = node
                .declare_parameter("declared_int", 1, ParameterOptions::default())
                .unwrap();
            assert_eq!(param.get(), 10);
            param.set(2);
            assert_eq!(param.get(), 2);
            // Redeclaring should fail
            assert!(node
                .declare_parameter("declared_int", 1, ParameterOptions::default())
                .is_err());
        }
        {
            // Parameter went out of scope, redeclaring should be OK and return command line
            // oveerride
            let param = node
                .declare_parameter("declared_int", 1, ParameterOptions::default())
                .unwrap();
            assert_eq!(param.get(), 10);
        }
        // After a declared parameter went out of scope and was cleared, it should still be
        // possible to use it as an undeclared parameter, type can now be changed
        assert!(node
            .use_undeclared_parameters()
            .get::<i64>("declared_int")
            .is_none());
        node.use_undeclared_parameters().set("declared_int", 1.0);
        assert_eq!(
            node.use_undeclared_parameters().get::<f64>("declared_int"),
            Some(1.0)
        );
        Ok(())
    }

    #[test]
    fn test_dynamic_undeclared_parameter() -> Result<(), RclrsError> {
        let ctx = Context::new([])?;
        let node = create_node(&ctx, "param_test_node")?;
        Ok(())
    }
}

mod override_map;
mod service;
mod value;

pub(crate) use override_map::*;
pub(crate) use service::*;
pub use value::*;

use crate::rcl_bindings::*;
use crate::{call_string_getter_with_handle, RclrsError};
use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};

#[derive(Clone, Debug)]
pub struct ParameterOptions {
    pub dynamic: bool,
    // TODO(luca) add int / float range
}

// TODO(luca) add From for rcl_interfaces::ParameterDescriptor message, but a manual implementation
// for the opposite since this struct does not contain all the fields required to populate the
// message
impl Default for ParameterOptions {
    fn default() -> Self {
        Self { dynamic: false }
    }
}

struct Parameter {
    value: Option<ParameterValue>,
    options: ParameterOptions,
}

pub struct ParameterInterface {
    _parameter_map: BTreeMap<String, Parameter>,
    _override_map: ParameterOverrideMap,
    _services: ParameterService,
}

impl ParameterInterface {
    pub fn new(
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
            _parameter_map: Default::default(),
            _override_map,
            _services,
        })
    }

    pub fn declare<T: ParameterVariant + Default>(
        &mut self,
        name: &str,
        options: ParameterOptions,
        default_value: Option<T>,
    ) -> Option<ParameterValue> {
        if let Some(param_override) = self._override_map.get(name) {
            // TODO(luca) rclcpp and rclpy allow the override to overwrite the type, maybe we
            // should return an error if the override is a different value than declared.
            // This implementation ignores the value and prints a warning but it is obviously not
            // final since a println! is not a proper warning
            if options.dynamic
                || std::mem::discriminant(param_override)
                    == std::mem::discriminant(&T::default().into())
            {
                self._parameter_map.insert(
                    name.to_owned(),
                    Parameter {
                        options,
                        value: Some(param_override.clone()),
                    },
                );
                return Some(param_override.clone());
            } else {
                println!("Mismatch in parameter override type for {}, ignoring", name);
            }
        }
        let value = default_value.map(|v| v.into());
        self._parameter_map.insert(
            name.to_owned(),
            Parameter {
                options,
                value: value.clone(),
            },
        );
        value
    }

    pub fn get<T: ParameterVariant>(&self, name: &str) -> Option<T> {
        self._parameter_map
            .get(name)
            .and_then(|p| p.value.clone().and_then(T::maybe_from))
    }

    // TODO(luca) either implement a new error or a new RclrsError variant
    pub fn set<T: ParameterVariant + Default>(
        &mut self,
        name: &str,
        value: T,
    ) -> Result<(), ()> {
        match self._parameter_map.get_mut(name) {
            Some(p) => {
                if p.options.dynamic
                    || p.value.is_none()
                    || p.value.as_ref().is_some_and(|p| {
                        std::mem::discriminant(p) == std::mem::discriminant(&T::default().into())
                    })
                {
                    // TODO(luca) design document requires ability to "try" setting parameters
                    // without actually doing it, this will be especially needed for setting
                    // parameters atomically
                    p.value = Some(value.into());
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

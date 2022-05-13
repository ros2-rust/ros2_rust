use crate::rcl_bindings::*;
use crate::{ParameterValue, RclrsError, ToResult};
use libc::c_char;
use std::collections::BTreeMap;
use std::ffi::CStr;

// Internal helper struct, iterator for rcl_params_t
struct RclParamsIter<'a> {
    node_name_ptrs: &'a [*mut c_char],
    rcl_node_params: &'a [rcl_node_params_t],
}

// Internal helper struct, iterator for rcl_node_params_t
struct RclNodeParamsIter<'a> {
    param_name_ptrs: &'a [*mut c_char],
    rcl_variants: &'a [rcl_variant_t],
}

impl<'a> Iterator for RclParamsIter<'a> {
    type Item = (String, &'a rcl_node_params_t);
    fn next(&mut self) -> Option<Self::Item> {
        let (next_node_param, rest) = self.rcl_node_params.split_first()?;
        self.rcl_node_params = rest;
        let (next_node_name_ptr, rest) = self.node_name_ptrs.split_first()?;
        self.node_name_ptrs = rest;
        debug_assert!(!next_node_name_ptr.is_null());
        // SAFETY: Pointer can be assumed to be valid. No lifetime issues due to immediate
        // conversion to owned string.
        let mut next_node_name = unsafe {
            CStr::from_ptr(*next_node_name_ptr)
                .to_string_lossy()
                .into_owned()
        };
        if !next_node_name.starts_with('/') {
            next_node_name.insert(0, '/');
        }
        Some((next_node_name, next_node_param))
    }
}

impl RclParamsIter<'_> {
    // This function is unsafe since the rcl_params argument might contain incorrect array sizes or
    // dangling pointers.
    pub unsafe fn new(rcl_params: *const rcl_params_t) -> Self {
        // The params will be null if none were given on the command line.
        // So, semantically, this can be treated like a struct with no parameters.
        if rcl_params.is_null() {
            Self {
                node_name_ptrs: &[],
                rcl_node_params: &[],
            }
        } else {
            let node_name_ptrs =
                std::slice::from_raw_parts((*rcl_params).node_names, (*rcl_params).num_nodes);
            let rcl_node_params =
                std::slice::from_raw_parts((*rcl_params).params, (*rcl_params).num_nodes);
            Self {
                node_name_ptrs,
                rcl_node_params,
            }
        }
    }
}

impl<'a> Iterator for RclNodeParamsIter<'a> {
    type Item = (String, &'a rcl_variant_t);
    fn next(&mut self) -> Option<Self::Item> {
        let (next_param_name_ptr, rest) = self.param_name_ptrs.split_first()?;
        self.param_name_ptrs = rest;
        let (next_rcl_variant, rest) = self.rcl_variants.split_first()?;
        self.rcl_variants = rest;
        // SAFETY: Pointer can be assumed to be valid. No lifetime issues due to immediate
        // conversion to owned string.
        let next_param_name = unsafe {
            CStr::from_ptr(*next_param_name_ptr)
                .to_string_lossy()
                .into_owned()
        };
        Some((next_param_name, next_rcl_variant))
    }
}

impl<'a> RclNodeParamsIter<'a> {
    // This function is unsafe since the rcl_node_params argument might contain incorrect array
    // sizes or dangling pointers.
    pub unsafe fn new(rcl_node_params: &'a rcl_node_params_t) -> Self {
        let param_name_ptrs =
            std::slice::from_raw_parts(rcl_node_params.parameter_names, rcl_node_params.num_params);
        let rcl_variants = std::slice::from_raw_parts(
            rcl_node_params.parameter_values,
            rcl_node_params.num_params,
        );
        Self {
            param_name_ptrs,
            rcl_variants,
        }
    }
}

/// A map of parameters for this node.
/// The key for this map is the parameter name.
/// A parameter override "overrides" the default value set from within the node, hence the name.
pub(crate) type ParameterOverrideMap = BTreeMap<String, ParameterValue>;

/// Similar to `rclcpp::detail::resolve_parameter_overrides`, but returns a map instead of
/// a vector.
///
/// This function is unsafe since the rcl_global_arguments argument might contain incorrect array
/// sizes or dangling pointers.
pub(crate) unsafe fn resolve_parameter_overrides(
    node_fqn: String,
    rcl_global_arguments: &rcl_arguments_t,
) -> Result<ParameterOverrideMap, RclrsError> {
    let mut map = BTreeMap::new();
    let mut rcl_params = std::ptr::null_mut();
    rcl_arguments_get_param_overrides(rcl_global_arguments, &mut rcl_params).ok()?;
    // Check for the /** node first, and later overwrite with the more specific node
    // parameters, if they exist
    for name_to_match in ["/**", node_fqn.as_str()] {
        for (node_name, node_params) in RclParamsIter::new(rcl_params) {
            if node_name == name_to_match {
                for (param_name, variant) in RclNodeParamsIter::new(node_params) {
                    let value = ParameterValue::from_rcl_variant(variant);
                    map.insert(param_name, value);
                }
            }
        }
    }
    rcl_yaml_node_struct_fini(rcl_params);
    Ok(map)
}

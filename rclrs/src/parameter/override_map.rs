use std::collections::BTreeMap;
use std::ffi::CStr;
use std::os::raw::c_char;

use crate::rcl_bindings::*;
use crate::{ParameterValue, RclrsError, ToResult};

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
/// A BTreeMap is used because it supports range queries, making it easy to fetch all parameters
/// with a given prefix.
pub(crate) type ParameterOverrideMap = BTreeMap<String, ParameterValue>;

/// Similar to `rclcpp::detail::resolve_parameter_overrides`, but returns a map instead of
/// a vector.
///
/// This function is unsafe since the rcl_global_arguments argument might contain incorrect array
/// sizes or dangling pointers.
pub(crate) unsafe fn resolve_parameter_overrides(
    node_fqn: &str,
    rcl_node_arguments: &rcl_arguments_t,
    rcl_global_arguments: &rcl_arguments_t,
) -> Result<ParameterOverrideMap, RclrsError> {
    let mut map = BTreeMap::new();
    for rcl_arguments in [rcl_global_arguments, rcl_node_arguments] {
        let mut rcl_params = std::ptr::null_mut();
        rcl_arguments_get_param_overrides(rcl_arguments, &mut rcl_params).ok()?;
        // Check for the /** node first, and later overwrite with the more specific node
        // parameters, if they exist
        for name_to_match in ["/**", node_fqn] {
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
    }
    Ok(map)
}

#[cfg(test)]
mod tests {
    use std::error::Error;
    use std::ffi::CString;
    use std::io::Write;

    use tempfile::NamedTempFile;

    use super::*;

    // These files have values for every possible four-bit number, with the four bits being
    // * `/**` global params
    // * named global params
    // * `/**` node params
    // * named node params
    const GLOBAL_PARAMS_FILE: &str = r#"
/**:
    ros__parameters:
        a: 1
        b: 1
        c: 1
        d: 1
        e: 1
        f: 1
        g: 1
        h: 1
/my_ns/my_node:
    ros__parameters:
        a: 2
        b: 2
        c: 2
        d: 2
        i: 2
        j: 2
        k: 2
        l: 2
"#;

    const NODE_PARAMS_FILE: &str = r#"
/my_ns/my_node:
    ros__parameters:
        a: 3
        b: 3
        e: 3
        f: 3
        i: 3
        j: 3
        m: 3
        n: 3
/my_ns/my_node:
    ros__parameters:
        a: 4
        c: 4
        e: 4
        g: 4
        i: 4
        k: 4
        m: 4
        o: 4
"#;

    // The returned rcl_arguments need to be manually finalized with rcl_arguments_fini
    fn convert_to_rcl_arguments(
        param_file_contents: &str,
    ) -> Result<rcl_arguments_t, Box<dyn Error>> {
        let mut params_file = NamedTempFile::new()?;
        write!(params_file, "{}", param_file_contents)?;
        let params_filename = params_file.path().display().to_string();
        let args = ["--ros-args", "--params-file", &params_filename]
            .into_iter()
            .map(CString::new)
            .collect::<Result<Vec<_>, _>>()?;
        let args_ptrs = args.iter().map(|s| s.as_ptr()).collect::<Vec<_>>();
        let mut rcl_arguments = unsafe { rcl_get_zero_initialized_arguments() };
        unsafe {
            rcl_parse_arguments(
                args_ptrs.len() as i32,
                args_ptrs.as_ptr(),
                rcutils_get_default_allocator(),
                &mut rcl_arguments,
            );
        }
        Ok(rcl_arguments)
    }

    #[test]
    fn test_resolve_parameter_overrides() -> Result<(), Box<dyn Error>> {
        let node_fqn = "/my_ns/my_node";
        let mut rcl_node_arguments = convert_to_rcl_arguments(NODE_PARAMS_FILE)?;
        let mut rcl_global_arguments = convert_to_rcl_arguments(GLOBAL_PARAMS_FILE)?;
        let overrides_map = unsafe {
            resolve_parameter_overrides(node_fqn, &rcl_node_arguments, &rcl_global_arguments)?
        };
        unsafe {
            rcl_arguments_fini(&mut rcl_node_arguments);
            rcl_arguments_fini(&mut rcl_global_arguments);
        }
        let values: Vec<_> = overrides_map
            .iter()
            .map(|(k, v)| (k.as_str(), v.clone()))
            .collect();
        assert_eq!(
            values,
            vec![
                ("a", ParameterValue::Integer(4)),
                ("b", ParameterValue::Integer(3)),
                ("c", ParameterValue::Integer(4)),
                ("d", ParameterValue::Integer(2)),
                ("e", ParameterValue::Integer(4)),
                ("f", ParameterValue::Integer(3)),
                ("g", ParameterValue::Integer(4)),
                ("h", ParameterValue::Integer(1)),
                ("i", ParameterValue::Integer(4)),
                ("j", ParameterValue::Integer(3)),
                ("k", ParameterValue::Integer(4)),
                ("l", ParameterValue::Integer(2)),
                ("m", ParameterValue::Integer(4)),
                ("n", ParameterValue::Integer(3)),
                ("o", ParameterValue::Integer(4)),
            ]
        );
        Ok(())
    }
}

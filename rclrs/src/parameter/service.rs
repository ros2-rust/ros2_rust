use std::collections::BTreeSet;
use std::sync::{Arc, Mutex};

use crate::vendor::rcl_interfaces::msg::rmw::*;
use crate::vendor::rcl_interfaces::srv::rmw::*;
use rosidl_runtime_rs::Sequence;

use super::ParameterMap;
use crate::parameter::{DeclaredValue, ParameterKind, ParameterStorage};
use crate::{rmw_request_id_t, Node, RclrsError, Service};

// The variables only exist to keep a strong reference to the services and are technically unused.
// What is used is the Weak that is stored in the node, and is upgraded when spinning.
pub struct ParameterService {
    #[allow(dead_code)]
    describe_parameters_service: Arc<Service<DescribeParameters>>,
    #[allow(dead_code)]
    get_parameter_types_service: Arc<Service<GetParameterTypes>>,
    #[allow(dead_code)]
    get_parameters_service: Arc<Service<GetParameters>>,
    #[allow(dead_code)]
    list_parameters_service: Arc<Service<ListParameters>>,
    #[allow(dead_code)]
    set_parameters_service: Arc<Service<SetParameters>>,
    #[allow(dead_code)]
    set_parameters_atomically_service: Arc<Service<SetParametersAtomically>>,
}

impl ParameterService {
    pub(crate) fn new(
        node: &Node,
        parameter_map: Arc<Mutex<ParameterMap>>,
    ) -> Result<Self, RclrsError> {
        let fqn = node.fully_qualified_name();
        // TODO(luca) make sure it is OK to have an Arc instead of a Weak here and cleanup on
        // destruction is made for the parameter map.
        let map = parameter_map.clone();
        let describe_parameters_service = node.create_service(
            &(fqn.clone() + "/describe_parameters"),
            move |_req_id: &rmw_request_id_t, req: DescribeParameters_Request| {
                let map = map.lock().unwrap();
                let descriptors = req
                    .names
                    .into_iter()
                    .map(|name| {
                        // TODO(luca) Remove conversion to string and implement Borrow
                        let storage = map.storage.get(name.to_cstr().to_str().unwrap())?;
                        let mut descriptor = match storage {
                            ParameterStorage::Declared(storage) => {
                                let (integer_range, floating_point_range) =
                                    storage.options.ranges.to_descriptor_ranges();
                                ParameterDescriptor {
                                    name,
                                    type_: Default::default(),
                                    description: storage.options._description.clone().into(),
                                    additional_constraints: storage
                                        .options
                                        ._constraints
                                        .clone()
                                        .into(),
                                    dynamic_typing: matches!(storage.kind, ParameterKind::Dynamic),
                                    read_only: matches!(storage.value, DeclaredValue::ReadOnly(_)),
                                    floating_point_range,
                                    integer_range,
                                }
                            }
                            ParameterStorage::Undeclared(_) => ParameterDescriptor {
                                name,
                                dynamic_typing: true,
                                ..Default::default()
                            },
                        };
                        descriptor.type_ = storage.to_parameter_type();
                        Some(descriptor)
                    })
                    .collect::<Option<_>>()
                    .unwrap_or_default();
                // TODO(luca) error logging if the service call failed
                DescribeParameters_Response { descriptors }
            },
        )?;
        let map = parameter_map.clone();
        let get_parameter_types_service = node.create_service(
            &(fqn.clone() + "/get_parameter_types"),
            move |_req_id: &rmw_request_id_t, req: GetParameterTypes_Request| {
                let map = map.lock().unwrap();
                let types = req
                    .names
                    .into_iter()
                    .map(|name| {
                        // TODO(luca) Remove conversion to string and implement Borrow
                        map.storage
                            .get(name.to_cstr().to_str().unwrap())
                            .map(|s| s.to_parameter_type())
                    })
                    .collect::<Option<_>>()
                    .unwrap_or_default();
                GetParameterTypes_Response { types }
            },
        )?;
        let map = parameter_map.clone();
        let get_parameters_service = node.create_service(
            &(fqn.clone() + "/get_parameters"),
            move |_req_id: &rmw_request_id_t, req: GetParameters_Request| {
                let map = map.lock().unwrap();
                let values = req
                    .names
                    .into_iter()
                    .map(|name| {
                        // TODO(luca) Remove conversion to string and implement Borrow
                        let storage = map.storage.get(name.to_cstr().to_str().unwrap())?;
                        match storage {
                            ParameterStorage::Declared(storage) => match &storage.value {
                                DeclaredValue::Mandatory(v) => {
                                    Some(v.read().unwrap().clone().into())
                                }
                                DeclaredValue::Optional(v) => Some(
                                    v.read()
                                        .unwrap()
                                        .clone()
                                        .map(|v| v.into())
                                        .unwrap_or_default(),
                                ),
                                DeclaredValue::ReadOnly(v) => Some(v.clone().into()),
                            },
                            ParameterStorage::Undeclared(value) => Some(value.clone().into()),
                        }
                    })
                    .collect::<Option<_>>()
                    .unwrap_or_default();
                GetParameters_Response { values }
            },
        )?;
        let map = parameter_map.clone();
        let list_parameters_service = node.create_service(
            &(fqn.clone() + "/list_parameters"),
            move |_req_id: &rmw_request_id_t, req: ListParameters_Request| {
                let check_parameter_name_depth = |substring: &[i8]| {
                    if req.depth == ListParameters_Request::DEPTH_RECURSIVE {
                        return true;
                    }
                    u64::try_from(substring.iter().filter(|c| **c == ('.' as i8)).count()).unwrap()
                        < req.depth
                };
                let map = map.lock().unwrap();
                let names: Sequence<_> = map
                    .storage
                    .keys()
                    .filter_map(|name| {
                        let name: rosidl_runtime_rs::String = name.clone().into();
                        if req.prefixes.len() == 0 && check_parameter_name_depth(&name[..]) {
                            return Some(name);
                        }
                        req.prefixes
                            .iter()
                            .any(|prefix| {
                                if name == *prefix {
                                    return true;
                                }
                                let mut prefix = prefix.clone();
                                prefix.extend(".".chars());
                                if name.len() > prefix.len()
                                    && name[0..prefix.len()] == prefix[0..]
                                    && check_parameter_name_depth(&name[prefix.len()..])
                                {
                                    return true;
                                }
                                false
                            })
                            .then_some(name)
                    })
                    .collect();
                let prefixes: BTreeSet<rosidl_runtime_rs::String> = names
                    .iter()
                    .filter_map(|name| {
                        let name = name.to_string();
                        if let Some(pos) = name.rfind('.') {
                            return Some(name[0..pos].into());
                            //return Some(name[0..pos].iter().map(|c| *c as char).collect());
                        }
                        None
                    })
                    .collect();
                // TODO(luca) populate prefixes in result
                ListParameters_Response {
                    result: ListParametersResult {
                        names,
                        prefixes: prefixes.into_iter().collect(),
                    },
                }
            },
        )?;
        let map = parameter_map.clone();
        let set_parameters_service = node.create_service(
            &(fqn.clone() + "/set_parameters"),
            move |_req_id: &rmw_request_id_t, req: SetParameters_Request| {
                let mut map = map.lock().unwrap();
                let results = req
                    .parameters
                    .into_iter()
                    .map(|param| {
                        let name = param.name.to_cstr().to_str().unwrap();
                        let value = map.validate_parameter_setting(name, param.value)?;
                        map.store_parameter(name.into(), value);
                        Ok(())
                    })
                    .map(|res| match res {
                        Ok(()) => SetParametersResult {
                            successful: true,
                            reason: Default::default(),
                        },
                        Err(e) => SetParametersResult {
                            successful: false,
                            reason: e,
                        },
                    })
                    .collect();
                SetParameters_Response { results }
            },
        )?;
        let set_parameters_atomically_service = node.create_service(
            &(fqn.clone() + "/set_parameters_atomically"),
            move |_req_id: &rmw_request_id_t, req: SetParametersAtomically_Request| {
                let mut map = parameter_map.lock().unwrap();
                let results = req
                    .parameters
                    .into_iter()
                    .map(|param| {
                        let name = param.name.to_cstr().to_str().unwrap();
                        let value = map.validate_parameter_setting(name, param.value)?;
                        Ok((name.into(), value))
                    })
                    .collect::<Result<Vec<_>, _>>();
                // Check if there was any error and update parameters accordingly
                let result = match results {
                    Ok(results) => {
                        for (name, value) in results.into_iter() {
                            map.store_parameter(name, value);
                        }
                        SetParametersResult {
                            successful: true,
                            reason: Default::default(),
                        }
                    }
                    Err(reason) => SetParametersResult {
                        successful: false,
                        reason,
                    },
                };
                SetParametersAtomically_Response { result }
            },
        )?;
        Ok(Self {
            describe_parameters_service,
            get_parameter_types_service,
            get_parameters_service,
            list_parameters_service,
            set_parameters_service,
            set_parameters_atomically_service,
        })
    }
}

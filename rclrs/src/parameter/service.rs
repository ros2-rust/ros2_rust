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

fn describe_parameters(
    req: DescribeParameters_Request,
    map: &ParameterMap,
) -> DescribeParameters_Response {
    let descriptors = req
        .names
        .into_iter()
        .map(|name| {
            let name = name.to_cstr().to_str().ok()?;
            let Some(storage) = map.storage.get(name) else {
                return map.allow_undeclared.then(|| ParameterDescriptor {
                    name: name.into(),
                    ..Default::default()
                });
            };
            let mut descriptor = match storage {
                ParameterStorage::Declared(storage) => {
                    let (integer_range, floating_point_range) =
                        storage.options.ranges.to_descriptor_ranges();
                    ParameterDescriptor {
                        name: name.into(),
                        type_: Default::default(),
                        description: storage.options.description.clone().into(),
                        additional_constraints: storage.options.constraints.clone().into(),
                        dynamic_typing: matches!(storage.kind, ParameterKind::Dynamic),
                        read_only: matches!(storage.value, DeclaredValue::ReadOnly(_)),
                        floating_point_range,
                        integer_range,
                    }
                }
                ParameterStorage::Undeclared(_) => ParameterDescriptor {
                    name: name.into(),
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
}

fn get_parameter_types(
    req: GetParameterTypes_Request,
    map: &ParameterMap,
) -> GetParameterTypes_Response {
    let types = req
        .names
        .into_iter()
        .map(|name| {
            let name = name.to_cstr().to_str().ok()?;
            map.storage
                .get(name)
                .map(|s| s.to_parameter_type())
                .or_else(|| {
                    map.allow_undeclared
                        .then_some(ParameterType::PARAMETER_NOT_SET)
                })
        })
        .collect::<Option<_>>()
        .unwrap_or_default();
    GetParameterTypes_Response { types }
}

fn get_parameters(req: GetParameters_Request, map: &ParameterMap) -> GetParameters_Response {
    let values = req
        .names
        .into_iter()
        .map(|name| {
            let name = name.to_cstr().to_str().ok()?;
            let Some(storage) = map.storage.get(name) else {
                return map.allow_undeclared.then(ParameterValue::default);
            };
            match storage {
                ParameterStorage::Declared(storage) => match &storage.value {
                    DeclaredValue::Mandatory(v) => Some(v.read().unwrap().clone().into()),
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
}

fn list_parameters(req: ListParameters_Request, map: &ParameterMap) -> ListParameters_Response {
    let check_parameter_name_depth = |substring: &[i8]| {
        if req.depth == ListParameters_Request::DEPTH_RECURSIVE {
            return true;
        }
        u64::try_from(substring.iter().filter(|c| **c == ('.' as i8)).count()).unwrap() < req.depth
    };
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
                        && name.starts_with(&prefix)
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
            }
            None
        })
        .collect();
    ListParameters_Response {
        result: ListParametersResult {
            names,
            prefixes: prefixes.into_iter().collect(),
        },
    }
}

fn set_parameters(req: SetParameters_Request, map: &mut ParameterMap) -> SetParameters_Response {
    let results = req
        .parameters
        .into_iter()
        .map(|param| {
            let Ok(name) = param.name.to_cstr().to_str() else {
                return SetParametersResult {
                    successful: false,
                    reason: "Failed parsing into UTF-8".into(),
                };
            };
            match map.validate_parameter_setting(name, param.value) {
                Ok(value) => {
                    map.store_parameter(name.into(), value);
                    SetParametersResult {
                        successful: true,
                        reason: Default::default(),
                    }
                }
                Err(e) => SetParametersResult {
                    successful: false,
                    reason: e.into(),
                },
            }
        })
        .collect();
    SetParameters_Response { results }
}

fn set_parameters_atomically(
    req: SetParametersAtomically_Request,
    map: &mut ParameterMap,
) -> SetParametersAtomically_Response {
    let results = req
        .parameters
        .into_iter()
        .map(|param| {
            let Ok(name) = param.name.to_cstr().to_str() else {
                return Err("Failed parsing into UTF-8".into());
            };
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
                describe_parameters(req, &map)
            },
        )?;
        let map = parameter_map.clone();
        let get_parameter_types_service = node.create_service(
            &(fqn.clone() + "/get_parameter_types"),
            move |_req_id: &rmw_request_id_t, req: GetParameterTypes_Request| {
                let map = map.lock().unwrap();
                get_parameter_types(req, &map)
            },
        )?;
        let map = parameter_map.clone();
        let get_parameters_service = node.create_service(
            &(fqn.clone() + "/get_parameters"),
            move |_req_id: &rmw_request_id_t, req: GetParameters_Request| {
                let map = map.lock().unwrap();
                get_parameters(req, &map)
            },
        )?;
        let map = parameter_map.clone();
        let list_parameters_service = node.create_service(
            &(fqn.clone() + "/list_parameters"),
            move |_req_id: &rmw_request_id_t, req: ListParameters_Request| {
                let map = map.lock().unwrap();
                list_parameters(req, &map)
            },
        )?;
        let map = parameter_map.clone();
        let set_parameters_service = node.create_service(
            &(fqn.clone() + "/set_parameters"),
            move |_req_id: &rmw_request_id_t, req: SetParameters_Request| {
                let mut map = map.lock().unwrap();
                set_parameters(req, &mut map)
            },
        )?;
        let set_parameters_atomically_service = node.create_service(
            &(fqn.clone() + "/set_parameters_atomically"),
            move |_req_id: &rmw_request_id_t, req: SetParametersAtomically_Request| {
                let mut map = parameter_map.lock().unwrap();
                set_parameters_atomically(req, &mut map)
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

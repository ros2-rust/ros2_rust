use std::sync::{Arc, Mutex};

use crate::vendor::rcl_interfaces::srv::rmw::*;
use crate::vendor::rcl_interfaces::msg::rmw::*;
use rosidl_runtime_rs::{BoundedSequence, Sequence, seq};

use crate::{rmw_request_id_t, Node, RclrsError, Service};
use crate::parameter::{DeclaredValue, ParameterRanges, ParameterKind, ParameterStorage};
use super::ParameterMap;

pub struct ParameterService {
    describe_parameters_service: Arc<Service<DescribeParameters>>,
    get_parameter_types_service: Arc<Service<GetParameterTypes>>,
    get_parameters_service: Arc<Service<GetParameters>>,
    list_parameters_service: Arc<Service<ListParameters>>,
    set_parameters_service: Arc<Service<SetParameters>>,
    set_parameters_atomically_service: Arc<Service<SetParametersAtomically>>,
}

// TODO(luca) should this be a method in the DeclaredStorage impl?
fn storage_to_parameter_type(storage: &ParameterStorage) -> u8 {
    match storage {
        ParameterStorage::Declared(s) => {
            match s.kind {
                ParameterKind::Bool => ParameterType::PARAMETER_BOOL,
                ParameterKind::Integer => ParameterType::PARAMETER_INTEGER,
                ParameterKind::Double => ParameterType::PARAMETER_DOUBLE,
                ParameterKind::String => ParameterType::PARAMETER_STRING,
                ParameterKind::ByteArray => ParameterType::PARAMETER_BYTE_ARRAY,
                ParameterKind::BoolArray => ParameterType::PARAMETER_BOOL_ARRAY,
                ParameterKind::IntegerArray => ParameterType::PARAMETER_INTEGER_ARRAY,
                ParameterKind::DoubleArray => ParameterType::PARAMETER_DOUBLE_ARRAY,
                ParameterKind::StringArray => ParameterType::PARAMETER_STRING_ARRAY,
                ParameterKind::Dynamic => {
                    match &s.value {
                        DeclaredValue::Mandatory(v) => v.read().unwrap().rcl_parameter_type(),
                        DeclaredValue::Optional(v) => v.read().unwrap().as_ref().map(|v| v.rcl_parameter_type()).unwrap_or(ParameterType::PARAMETER_NOT_SET),
                        DeclaredValue::ReadOnly(v) => v.rcl_parameter_type(),
                    }
                }
                
            }
        }
        ParameterStorage::Undeclared(value) => {
            value.rcl_parameter_type()
        }
    }
}

// TODO(luca) should this be a methond in the ParameterRanges impl?
fn parameter_ranges_to_descriptor_ranges(ranges: &ParameterRanges) -> (BoundedSequence<IntegerRange, 1>, BoundedSequence<FloatingPointRange, 1>) {
    let int_range = ranges.integer.as_ref().map(|range| {
        // Converting step to a positive value is safe because declaring a parameter with a
        // negative step is not allowed.
        // TODO(luca) explore changing step into a positive value in the generic definition to
        // make negative steps a compile error.
        if range.lower.is_none() && range.upper.is_none() && range.step.is_none() {
            // It's a default range, just return a default
            Default::default()
        } else {
            seq![1 # IntegerRange {
                from_value: range.lower.unwrap_or(i64::MIN),
                to_value: range.upper.unwrap_or(i64::MAX),
                step: range.step.unwrap_or(0).try_into().unwrap(),
            }]
        }
    })
    .unwrap_or_default();
    let float_range = ranges.float.as_ref().map(|range| {
        // TODO(luca) Double check whether we should use MIN/MAX or INFINITY/NEG_INFINITY
        if range.lower.is_none() && range.upper.is_none() && range.step.is_none() {
            Default::default()
        } else {
            seq![1 # FloatingPointRange {
                from_value: range.lower.unwrap_or(f64::MIN),
                to_value: range.upper.unwrap_or(f64::MAX),
                step: range.step.unwrap_or(0.0),
            }]
        }
    })
    .unwrap_or_default();
    (int_range, float_range)
}

impl ParameterService {
    pub(crate) fn new(node: &Node, parameter_map: Arc<Mutex<ParameterMap>>) -> Result<Self, RclrsError> {
        // TODO(luca) node fully qualified name for the service topic prefix
        let fqn = node.fully_qualified_name();
        // TODO(luca) make sure it is OK to have an Arc instead of a Weak here and cleanup on
        // destruction is made for the parameter map.
        let map = parameter_map.clone();
        let describe_parameters_service = node.create_service(
            "describe_parameters",
            move |req_id: &rmw_request_id_t, req: DescribeParameters_Request| {
                // TODO(luca) look at the request and filter names
                let map = map.lock().unwrap();
                let descriptors = req.names.iter().map(|name| {
                    // TODO(luca) Remove conversion to string and implement Borrow
                    let storage = map.storage.get(name.to_cstr().to_str().unwrap())?;
                    let mut descriptor = match storage {
                        ParameterStorage::Declared(storage) => {
                            let (integer_range, floating_point_range) = parameter_ranges_to_descriptor_ranges(&storage.options.ranges);
                            ParameterDescriptor {
                                name: name.clone().into(),
                                type_: Default::default(),
                                description: storage.options._description.clone().into(),
                                additional_constraints: storage.options._constraints.clone().into(),
                                dynamic_typing: matches!(storage.kind, ParameterKind::Dynamic),
                                read_only: matches!(storage.value, DeclaredValue::ReadOnly(_)),
                                floating_point_range,
                                integer_range,
                            }
                        }
                        ParameterStorage::Undeclared(value) => {
                            ParameterDescriptor {
                                name: name.clone().into(),
                                dynamic_typing: true,
                                ..Default::default()
                            }
                        }
                    };
                    descriptor.type_ = storage_to_parameter_type(storage);
                    Some(descriptor)
                })
                .collect::<Option<_>>()
                .unwrap_or_default();
                // TODO(luca) error logging if the service call failed
                DescribeParameters_Response {
                    descriptors,
                }
            },
        )?;
        let map = parameter_map.clone();
        let get_parameter_types_service = node.create_service(
            "get_parameter_types",
            move |req_id: &rmw_request_id_t, req: GetParameterTypes_Request| {
                // TODO(luca) look at the request and filter names
                let types = map.lock().unwrap().storage.iter().map(|(name, storage)| {
                    storage_to_parameter_type(storage)
                })
                .collect();
                GetParameterTypes_Response {
                    types,
                }
            },
        )?;
        let map = parameter_map.clone();
        let get_parameters_service = node.create_service(
            "get_parameters",
            move |req_id: &rmw_request_id_t, req: GetParameters_Request| {
                GetParameters_Response {
                    values: seq![]
                }
            },
        )?;
        let map = parameter_map.clone();
        let list_parameters_service = node.create_service(
            "list_parameters",
            move |req_id: &rmw_request_id_t, req: ListParameters_Request| {
                ListParameters_Response {
                    result: ListParametersResult::default()
                }
            },
        )?;
        let map = parameter_map.clone();
        let set_parameters_service = node.create_service(
            "set_parameters",
            move |req_id: &rmw_request_id_t, req: SetParameters_Request| {
                SetParameters_Response {
                    results: seq![]
                }
            },
        )?;
        let set_parameters_atomically_service = node.create_service(
            "set_parameters_atomically",
            move |req_id: &rmw_request_id_t, req: SetParametersAtomically_Request| {
                SetParametersAtomically_Response {
                    result: SetParametersResult::default()
                }
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

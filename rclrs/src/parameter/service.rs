use std::sync::{Arc, Weak, Mutex};

use crate::vendor::rcl_interfaces::srv::rmw::*;
use crate::vendor::rcl_interfaces::msg::rmw::*;
use rosidl_runtime_rs::{Sequence, seq};

use crate::{rmw_request_id_t, Node, RclrsError, Service, ServiceBase};
use crate::rcl_bindings::rcl_node_t;

pub struct ParameterService {
    describe_parameters_service: Arc<Service<DescribeParameters>>,
    get_parameter_types_service: Arc<Service<GetParameterTypes>>,
    get_parameters_service: Arc<Service<GetParameters>>,
    list_parameters_service: Arc<Service<ListParameters>>,
    set_parameters_service: Arc<Service<SetParameters>>,
    set_parameters_atomically_service: Arc<Service<SetParametersAtomically>>,
}

impl ParameterService {
    pub fn new(rcl_node_mtx: Arc<Mutex<rcl_node_t>>) -> Result<Self, RclrsError> {
        let describe_parameters_service = Service::new(Arc::clone(&rcl_node_mtx),
            "describe_parameters",
            |req_id: &rmw_request_id_t, req: DescribeParameters_Request| {
                DescribeParameters_Response {
                    descriptors: seq![]
                }
            },
        )?;
        let get_parameter_types_service = Service::new(Arc::clone(&rcl_node_mtx),
            "get_parameter_types",
            |req_id: &rmw_request_id_t, req: GetParameterTypes_Request| {
                GetParameterTypes_Response {
                    types: seq![]
                }
            },
        )?;
        let get_parameters_service = Service::new(Arc::clone(&rcl_node_mtx),
            "get_parameters",
            |req_id: &rmw_request_id_t, req: GetParameters_Request| {
                GetParameters_Response {
                    values: seq![]
                }
            },
        )?;
        let list_parameters_service = Service::new(Arc::clone(&rcl_node_mtx),
            "list_parameters",
            |req_id: &rmw_request_id_t, req: ListParameters_Request| {
                ListParameters_Response {
                    result: ListParametersResult::default()
                }
            },
        )?;
        let set_parameters_service = Service::new(Arc::clone(&rcl_node_mtx),
            "set_parameters",
            |req_id: &rmw_request_id_t, req: SetParameters_Request| {
                SetParameters_Response {
                    results: seq![]
                }
            },
        )?;
        let set_parameters_atomically_service = Service::new(Arc::clone(&rcl_node_mtx),
            "set_parameters_atomically",
            |req_id: &rmw_request_id_t, req: SetParametersAtomically_Request| {
                SetParametersAtomically_Response {
                    result: SetParametersResult::default()
                }
            },
        )?;
        Ok(Self {
            describe_parameters_service: Arc::new(describe_parameters_service),
            get_parameter_types_service: Arc::new(get_parameter_types_service),
            get_parameters_service: Arc::new(get_parameters_service),
            list_parameters_service: Arc::new(list_parameters_service),
            set_parameters_service: Arc::new(set_parameters_service),
            set_parameters_atomically_service: Arc::new(set_parameters_atomically_service),
        })
    }

    pub fn services(&self) -> Vec<Weak<dyn ServiceBase>> {
        vec![
            Arc::downgrade(&self.describe_parameters_service) as Weak<dyn ServiceBase>,
            Arc::downgrade(&self.get_parameter_types_service) as Weak<dyn ServiceBase>,
            Arc::downgrade(&self.get_parameters_service) as Weak<dyn ServiceBase>,
            Arc::downgrade(&self.list_parameters_service) as Weak<dyn ServiceBase>,
            Arc::downgrade(&self.set_parameters_service) as Weak<dyn ServiceBase>,
            Arc::downgrade(&self.set_parameters_atomically_service) as Weak<dyn ServiceBase>,
        ]
    }
}

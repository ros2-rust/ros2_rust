@{
req_res_specs = []

for subfolder, service in srv_specs:
    req_res_specs.append((subfolder, service.request_message))
    req_res_specs.append((subfolder, service.response_message))
}@

@{
TEMPLATE(
    'msg_idiomatic.rs.em',
    package_name=package_name, interface_path=interface_path,
    msg_specs=req_res_specs,
    get_rs_name=get_rs_name, get_rmw_rs_type=get_rmw_rs_type,
    get_idiomatic_rs_type=get_idiomatic_rs_type,
    constant_value_to_rs=constant_value_to_rs)
}@

@[for subfolder, srv_spec in srv_specs]

@{
type_name = srv_spec.namespaced_type.name
}@

#[link(name = "@(package_name)__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__@(package_name)__@(subfolder)__@(type_name)() -> *const std::os::raw::c_void;
}

// Corresponds to @(package_name)__@(subfolder)__@(type_name)
pub struct @(type_name);

impl rosidl_runtime_rs::Service for @(type_name) {
  type Request = crate::@(subfolder)::@(type_name)_Request;
  type Response = crate::@(subfolder)::@(type_name)_Response;

  fn get_type_support() -> *const std::os::raw::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__@(package_name)__@(subfolder)__@(type_name)() }
  }
}

@[end for]

pub mod rmw {
@{
TEMPLATE(
    'msg_rmw.rs.em',
    package_name=package_name, interface_path=interface_path,
    msg_specs=req_res_specs,
    get_rs_name=get_rs_name, get_rmw_rs_type=get_rmw_rs_type,
    get_idiomatic_rs_type=get_idiomatic_rs_type,
    constant_value_to_rs=constant_value_to_rs)
}@

@[for subfolder, srv_spec in srv_specs]

@{
type_name = srv_spec.namespaced_type.name
}@

  #[link(name = "@(package_name)__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__@(package_name)__@(subfolder)__@(type_name)() -> *const std::os::raw::c_void;
  }

  // Corresponds to @(package_name)__@(subfolder)__@(type_name)
  pub struct @(type_name);

  impl rosidl_runtime_rs::Service for @(type_name) {
    type Request = crate::@(subfolder)::rmw::@(type_name)_Request;
    type Response = crate::@(subfolder)::rmw::@(type_name)_Response;

    fn get_type_support() -> *const std::os::raw::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__@(package_name)__@(subfolder)__@(type_name)() }
    }
  }

@[end for]

}  // mod rmw

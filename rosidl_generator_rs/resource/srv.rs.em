@{
TEMPLATE(
    'srv_idiomatic.rs.em',
    package_name=package_name, interface_path=interface_path,
    srv_specs=srv_specs,
    get_rs_name=get_rs_name, get_rmw_rs_type=get_rmw_rs_type,
    pre_field_serde=pre_field_serde,
    get_idiomatic_rs_type=get_idiomatic_rs_type,
    constant_value_to_rs=constant_value_to_rs)
}

pub mod rmw {
@{
TEMPLATE(
    'srv_rmw.rs.em',
    package_name=package_name, interface_path=interface_path,
    srv_specs=srv_specs,
    get_rs_name=get_rs_name, get_rmw_rs_type=get_rmw_rs_type,
    pre_field_serde=pre_field_serde,
    get_idiomatic_rs_type=get_idiomatic_rs_type,
    constant_value_to_rs=constant_value_to_rs)
}@
}  // mod rmw

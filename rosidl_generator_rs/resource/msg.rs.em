pub mod rmw {
@{
TEMPLATE(
    'msg_rmw.rs.em',
    package_name=package_name, interface_path=interface_path,
    msg_specs=msg_specs,
    get_rs_name=get_rs_name, get_rmw_rs_type=get_rmw_rs_type,
    get_idiomatic_rs_type=get_idiomatic_rs_type)
}@
}  // mod rmw

@{
TEMPLATE(
    'msg_idiomatic.rs.em',
    package_name=package_name, interface_path=interface_path,
    msg_specs=msg_specs,
    get_rs_name=get_rs_name, get_rmw_rs_type=get_rmw_rs_type,
    get_idiomatic_rs_type=get_idiomatic_rs_type)
}@

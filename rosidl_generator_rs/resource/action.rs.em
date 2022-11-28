@{
action_msg_specs = []

for subfolder, action in action_specs:
    action_msg_specs.append((subfolder, action.goal))
    action_msg_specs.append((subfolder, action.result))
    action_msg_specs.append((subfolder, action.feedback))
    action_msg_specs.append((subfolder, action.feedback_message))

action_srv_specs = []

for subfolder, action in action_specs:
    action_srv_specs.append((subfolder, action.send_goal_service))
    action_srv_specs.append((subfolder, action.get_result_service))
}@

pub mod rmw {
@{
TEMPLATE(
    'msg_rmw.rs.em',
    package_name=package_name, interface_path=interface_path,
    msg_specs=action_msg_specs,
    get_rs_name=get_rs_name, get_rmw_rs_type=get_rmw_rs_type,
    get_idiomatic_rs_type=get_idiomatic_rs_type,
    constant_value_to_rs=constant_value_to_rs)
}@
}  // mod rmw

@{
TEMPLATE(
    'msg_idiomatic.rs.em',
    package_name=package_name, interface_path=interface_path,
    msg_specs=action_msg_specs,
    get_rs_name=get_rs_name, get_rmw_rs_type=get_rmw_rs_type,
    get_idiomatic_rs_type=get_idiomatic_rs_type,
    constant_value_to_rs=constant_value_to_rs)
}@

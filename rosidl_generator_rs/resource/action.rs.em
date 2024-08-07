@{
from rosidl_parser.definition import (
    ACTION_FEEDBACK_MESSAGE_SUFFIX,
    ACTION_FEEDBACK_SUFFIX,
    ACTION_GOAL_SERVICE_SUFFIX,
    ACTION_GOAL_SUFFIX,
    ACTION_RESULT_SERVICE_SUFFIX,
    ACTION_RESULT_SUFFIX,
)

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
    pre_field_serde=pre_field_serde,
    get_idiomatic_rs_type=get_idiomatic_rs_type,
    constant_value_to_rs=constant_value_to_rs)

TEMPLATE(
    'srv_rmw.rs.em',
    package_name=package_name, interface_path=interface_path,
    srv_specs=action_srv_specs,
    get_rs_name=get_rs_name, get_rmw_rs_type=get_rmw_rs_type,
    pre_field_serde=pre_field_serde,
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
    pre_field_serde=pre_field_serde,
    get_idiomatic_rs_type=get_idiomatic_rs_type,
    constant_value_to_rs=constant_value_to_rs)
}@

@{
TEMPLATE(
    'srv_idiomatic.rs.em',
    package_name=package_name, interface_path=interface_path,
    srv_specs=action_srv_specs,
    get_rs_name=get_rs_name, get_rmw_rs_type=get_rmw_rs_type,
    pre_field_serde=pre_field_serde,
    get_idiomatic_rs_type=get_idiomatic_rs_type,
    constant_value_to_rs=constant_value_to_rs)
}@

@[for subfolder, action_spec in action_specs]

@{
type_name = action_spec.namespaced_type.name
}@

#[link(name = "@(package_name)__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__@(package_name)__@(subfolder)__@(type_name)() -> *const std::os::raw::c_void;
}

// Corresponds to @(package_name)__@(subfolder)__@(type_name)
pub struct @(type_name);

impl rosidl_runtime_rs::Action for @(type_name) {
  type Goal = crate::@(subfolder)::rmw::@(type_name)@(ACTION_GOAL_SUFFIX);
  type Result = crate::@(subfolder)::rmw::@(type_name)@(ACTION_RESULT_SUFFIX);
  type Feedback = crate::@(subfolder)::rmw::@(type_name)@(ACTION_FEEDBACK_SUFFIX);

  fn get_type_support() -> *const std::os::raw::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_action_type_support_handle__@(package_name)__@(subfolder)__@(type_name)() }
  }
}

impl rosidl_runtime_rs::ActionImpl for @(type_name) {
  type GoalStatusMessage = action_msgs::msg::rmw::GoalStatusArray;
  type FeedbackMessage = crate::@(subfolder)::rmw::@(type_name)@(ACTION_FEEDBACK_MESSAGE_SUFFIX);

  type SendGoalService = crate::@(subfolder)::rmw::@(type_name)@(ACTION_GOAL_SERVICE_SUFFIX);
  type CancelGoalService = action_msgs::srv::rmw::CancelGoal;
  type GetResultService = crate::@(subfolder)::rmw::@(type_name)@(ACTION_RESULT_SERVICE_SUFFIX);
}

@[end for]

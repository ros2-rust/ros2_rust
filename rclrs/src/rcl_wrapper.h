#include <rcl/graph.h>
#include <rcl/rcl.h>
#include <rcl_action/types.h>
#include <rcl_action/goal_handle.h>
#include <rcl_yaml_param_parser/parser.h>
#include <rcutils/error_handling.h>
#include <rmw/types.h>
#include <rosidl_typesupport_introspection_c/field_types.h>
#include <rosidl_typesupport_introspection_c/message_introspection.h>

const size_t rmw_gid_storage_size_constant = RMW_GID_STORAGE_SIZE;
const size_t rcl_action_uuid_size_constant = UUID_SIZE;
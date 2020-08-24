#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/message_type_support_struct.h"

@{
from rosidl_parser.definition import AbstractGenericString
from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import Array
from rosidl_parser.definition import BasicType
}@

@[for subfolder, msg_spec in msg_specs]@
@{
type_name = msg_spec.structure.namespaced_type.name
c_fields = []
for member in msg_spec.structure.members:
    if type(member.type) is Array:
        pass
    else:
        if isinstance(member.type, BasicType) or isinstance(member.type, AbstractGenericString):
            c_fields.append("%s %s" % (get_c_type(member.type), member.name))
        else:
            pass

msg_normalized_type = get_rs_type(msg_spec.structure.namespaced_type).replace('::', '__')
}@

#include "@(package_name)/@(subfolder)/@(convert_camel_case_to_lower_case_underscore(type_name)).h"

uintptr_t @(package_name)_msg_@(convert_camel_case_to_lower_case_underscore(type_name))_get_type_support() {
    return (uintptr_t)ROSIDL_GET_MSG_TYPE_SUPPORT(@(package_name), @(subfolder), @(msg_spec.structure.namespaced_type.name));
}

uintptr_t @(package_name)_msg_@(convert_camel_case_to_lower_case_underscore(type_name))_get_native_message(
  @(', '.join(c_fields))) {
      @(msg_normalized_type) *ros_message = @(msg_normalized_type)__create();
@[for member in msg_spec.structure.members]@
@[    if isinstance(member.type, Array)]@
@[    elif isinstance(member.type, AbstractGenericString)]@
      rosidl_runtime_c__String__assign(&(ros_message->@(member.name)), @(member.name));
@[    elif isinstance(member.type, BasicType)]@
      ros_message->@(member.name) = @(member.name);
@[    end if]@
@[end for]@
    return (uintptr_t)ros_message;
}

void @(package_name)_msg_@(convert_camel_case_to_lower_case_underscore(type_name))_destroy_native_message(void * raw_ros_message) {
      @(msg_normalized_type) * ros_message = raw_ros_message;
      @(msg_normalized_type)__destroy(ros_message);
}

@[for member in msg_spec.structure.members]@
@(get_c_type(member.type)) @(package_name)_msg_@(convert_camel_case_to_lower_case_underscore(type_name))_@(member.name)_read_handle(uintptr_t message_handle) {
@[    if isinstance(member.type, Array)]@
    (void)message_handle;
    return 0;
@[    elif isinstance(member.type, AbstractGenericString)]@
    @(msg_normalized_type) * ros_message = (@(msg_normalized_type) *)message_handle;
    return ros_message->@(member.name).data;
@[    elif isinstance(member.type, BasicType)]@
    @(msg_normalized_type) * ros_message = (@(msg_normalized_type) *)message_handle;
    return ros_message->@(member.name);
@[    elif isinstance(member.type, AbstractNestedType)]@
    @(msg_normalized_type) * ros_message = (@(msg_normalized_type) *)message_handle;
    return (@(get_c_type(member.type)))&ros_message->@(member.name);
@[    else]@
    (void)message_handle;
    return 0;
@[    end if]@
}
@[end for]@

@[end for]

#include "rosidl_generator_c/message_type_support_struct.h"

#include "rosidl_generator_c/string.h"
#include "rosidl_generator_c/string_functions.h"
#include <assert.h>

@[for subfolder, msg_spec in msg_specs]@
@{
type_name = msg_spec.base_type.type
c_fields = []
for field in msg_spec.fields:
    c_fields.append("%s %s" % (get_c_type(field.type), field.name))
    if field.type.is_array:
        c_fields.append("size_t %s__len" % field.name)


msg_normalized_type = get_normalized_type(msg_spec.base_type, subfolder=subfolder)
}@

#include "@(msg_spec.base_type.pkg_name)/@(subfolder)/@(convert_camel_case_to_lower_case_underscore(type_name)).h"

uintptr_t @(package_name)_msg_@(convert_camel_case_to_lower_case_underscore(type_name))_get_type_support() {
    return (uintptr_t)ROSIDL_GET_MSG_TYPE_SUPPORT(@(msg_spec.base_type.pkg_name), @(subfolder), @(msg_spec.msg_name));
}

uintptr_t @(package_name)_msg_@(convert_camel_case_to_lower_case_underscore(type_name))_get_native_message(
  @(',\n  '.join(c_fields))
) {
    @(msg_normalized_type) * ros_message = @(msg_normalized_type)__create();
@[for field in msg_spec.fields]@
@[    if field.type.is_array]@
@[        if field.type.is_primitive_type() and field.type.is_fixed_size_array()]@
    assert(@(field.name)__len == @(field.type.array_size));
    memcpy(ros_message->@(field.name), @(field.name), @(field.type.array_size));
@[        else]@
    ros_message->@(field.name).data = @(field.name);
    ros_message->@(field.name).size = @(field.name)__len;
@[        end if]@
@[    else]@
@[        if field.type.is_primitive_type()]@
@[            if field.type.type == 'string']@
    rosidl_generator_c__String__assign(&(ros_message->@(field.name)), @(field.name));
@[            else]@
    ros_message->@(field.name) = @(field.name);
@[            end if]@
@[        else]@
    memcpy(&(ros_message->@(field.name)), (@(get_normalized_type(field.type))*) @(field.name), sizeof(ros_message->@(field.name)));
@[        end if]@
@[    end if]@
@[end for]@
    return (uintptr_t)ros_message;
}

void @(package_name)_msg_@(convert_camel_case_to_lower_case_underscore(type_name))_destroy_native_message(void * raw_ros_message) {
      @(msg_normalized_type) * ros_message = raw_ros_message;
      @(msg_normalized_type)__destroy(ros_message);
}

@[for field in msg_spec.fields]@
@[    if field.type.is_array]@
@(get_c_type(field.type)) @(package_name)_msg_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(uintptr_t message_handle, size_t* size) {
@[        if field.type.is_primitive_type() and field.type.is_fixed_size_array()]@
    @(msg_normalized_type) * ros_message = (@(msg_normalized_type) *)message_handle;
    return ros_message->@(field.name);
@[        else]@
    @(msg_normalized_type) * ros_message = (@(msg_normalized_type) *)message_handle;
    *size = ros_message->@(field.name).size;
    return (uintptr_t)(ros_message->@(field.name).data);
@[        end if]@
@[    else]@
@(get_c_type(field.type)) @(package_name)_msg_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(uintptr_t message_handle) {
@[        if field.type.is_primitive_type()]@
@[            if field.type.type == 'string']@
    @(msg_normalized_type) * ros_message = (@(msg_normalized_type) *)message_handle;
    return ros_message->@(field.name).data;
@[            else]@
    @(msg_normalized_type) * ros_message = (@(msg_normalized_type) *)message_handle;
    return ros_message->@(field.name);
@[            end if]@
@[        else]@
    @(msg_normalized_type) * ros_message = (@(msg_normalized_type) *)message_handle;
    return (uintptr_t)(&(ros_message->@(field.name)));
@[        end if]@
@[    end if]@
}
@[end for]@

@[end for]

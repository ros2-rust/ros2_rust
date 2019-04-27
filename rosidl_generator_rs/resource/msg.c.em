#include "rosidl_generator_c/message_type_support_struct.h"

#include <string.h>

@{
have_not_included_primitive_arrays = True
have_not_included_string = True
nested_array_dict = {}
}@

@[for subfolder, msg_spec in msg_specs]@
@{
type_name = msg_spec.base_type.type
c_fields = []
for field in msg_spec.fields:
    if is_dynamic_array(field):
        c_fields.append("size_t %s__len" % field.name)
    if not is_nested(field):
        c_fields.append("%s %s" % (get_c_type(field), field.name))

msg_normalized_type = get_normalized_type(msg_spec.base_type, subfolder=subfolder)
}@
@
@[for field in msg_spec.fields]@
@[    if is_array(field) and have_not_included_primitive_arrays]@
@{have_not_included_primitive_arrays = False}@
#include <rosidl_generator_c/primitives_sequence.h>
#include <rosidl_generator_c/primitives_sequence_functions.h>

@[    end if]@
@[    if (is_single_string(field) or is_string_array(field)) and have_not_included_string]@
@{have_not_included_string = False}@
#include <rosidl_generator_c/string.h>
#include <rosidl_generator_c/string_functions.h>

@[    end if]@
@{
if is_nested_array(field):
    if field.type.type not in nested_array_dict:
        nested_array_dict[field.type.type] = field.type.pkg_name
}@
@[end for]@
@[if nested_array_dict != {}]@
@[    for key in nested_array_dict]@
#include <@(nested_array_dict[key])/msg/@convert_camel_case_to_lower_case_underscore(key)__functions.h>
@[    end for]@

@[end if]@
@
#include "@(msg_spec.base_type.pkg_name)/@(subfolder)/@(convert_camel_case_to_lower_case_underscore(type_name)).h"

uintptr_t @(package_name)_msg_@(convert_camel_case_to_lower_case_underscore(type_name))_get_type_support() {
    return (uintptr_t)ROSIDL_GET_MSG_TYPE_SUPPORT(@(msg_spec.base_type.pkg_name), @(subfolder), @(msg_spec.msg_name));
}

uintptr_t @(package_name)_msg_@(convert_camel_case_to_lower_case_underscore(type_name))_create_native_message() {
    @(msg_normalized_type) * ros_message = @(msg_normalized_type)__create();
    return (uintptr_t)ros_message;
}

void @(package_name)_msg_@(convert_camel_case_to_lower_case_underscore(type_name))_get_native_message_at(
    @(',\n    '.join(["%s * ros_message" % msg_normalized_type] + c_fields))
) {
@[for field in msg_spec.fields]@
@
@[    if is_static_string_array(field)]@
@[        for i in range(0, field.type.array_size)]@
    rosidl_generator_c__String__assign(&ros_message->@(field.name)[@(i)], @(field.name)[@(i)]);
@[        end for]@
@
@[    elif is_static_primitive_array(field)]@
    memcpy(ros_message->@(field.name), @(field.name), @(field.type.array_size) * sizeof(@get_builtin_c_type(field.type.type)));
@
@[    elif is_dynamic_string_array(field)]@
    rosidl_generator_c__String__Sequence__init(&(ros_message->@(field.name)), @(field.name)__len);
    for (uint i = 0; i < @(field.name)__len; ++i) {
        rosidl_generator_c__String__assign(&(ros_message->@(field.name).data[i]), @(field.name)[i]);
    }
@
@[    elif is_dynamic_primitive_array(field)]@
    rosidl_generator_c__@(field.type.type)__Sequence__init(&(ros_message->@(field.name)), @(field.name)__len);
    memcpy(ros_message->@(field.name).data, (void*) @(field.name), @(field.name)__len * sizeof(@get_builtin_c_type(field.type.type)));
@
@[    elif is_dynamic_nested_array(field)]@
    @(get_normalized_type(field.type))__Sequence__init(&(ros_message->@(field.name)), @(field.name)__len);
@
@[    elif is_single_string(field)]@
    rosidl_generator_c__String__assign(&(ros_message->@(field.name)), @(field.name));
@
@[    elif is_single_primitive(field)]@
    ros_message->@(field.name) = @(field.name);
@[    end if]@
@[end for]@
}

void @(package_name)_msg_@(convert_camel_case_to_lower_case_underscore(type_name))_destroy_native_message(void * raw_ros_message) {
    @(msg_normalized_type) * ros_message = (@(msg_normalized_type) *)raw_ros_message;
    @(msg_normalized_type)__destroy(ros_message);
}
@
@[for field in msg_spec.fields]@
@[    if is_dynamic_array(field)]
size_t @(package_name)_msg_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_array_size(uintptr_t message_handle) {
    @(msg_normalized_type) * ros_message = (@(msg_normalized_type) *)message_handle;
    return ros_message->@(field.name).size;
}
@[    end if]@

@[    if is_nested_array(field) or is_string_array(field)]@
void @(package_name)_msg_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(uintptr_t message_handle, uintptr_t* item_handles) {
@[    else]@
@(get_c_type(field)) @(package_name)_msg_@(convert_camel_case_to_lower_case_underscore(type_name))_@(field.name)_read_handle(uintptr_t message_handle) {
@[    end if]@
@
    @(msg_normalized_type) * ros_message = (@(msg_normalized_type) *)message_handle;
@
@[    if is_static_nested_array(field)]@
@[        for i in range(field.type.array_size)]@
    item_handles[@(i)] = (uintptr_t)(&(ros_message->@(field.name)[@(i)]));
@[        end for]@
@
@[    elif is_dynamic_nested_array(field)]@
    for(size_t i = 0; i < ros_message->@(field.name).size; ++i) {
        item_handles[i] = (uintptr_t)(&(ros_message->@(field.name).data[i]));
    }
@
@[    elif is_static_string_array(field)]@
@[        for i in range(field.type.array_size)]@
    item_handles[@(i)] = (uintptr_t)(ros_message->@(field.name)[@(i)].data);
@[        end for]@
@
@[    elif is_dynamic_string_array(field)]@
    for(size_t i = 0; i < ros_message->@(field.name).size; ++i) {
        item_handles[i] = (uintptr_t)(ros_message->@(field.name).data[i].data);
    }
@
@[    elif is_dynamic_primitive_array(field) or is_single_string(field)]@
    return ros_message->@(field.name).data;
@
@[    elif is_static_primitive_array(field) or is_single_primitive(field)]@
    return ros_message->@(field.name);
@
@[    else]@
    return (uintptr_t)(&(ros_message->@(field.name)));
@[    end if]@
}
@[end for]
@[end for]

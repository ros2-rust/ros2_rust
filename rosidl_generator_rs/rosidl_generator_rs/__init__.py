# Copyright 2016-2017 Esteve Fernandez <esteve@apache.org>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from rosidl_cmake import convert_camel_case_to_lower_case_underscore
from rosidl_cmake import expand_template
from rosidl_cmake import get_newest_modification_time
from rosidl_cmake import read_generator_arguments
from rosidl_parser import parse_message_file
from rosidl_parser import parse_service_file


# Taken from http://stackoverflow.com/a/6425628
def convert_lower_case_underscore_to_camel_case(word):
    return "".join(x.capitalize() or "_" for x in word.split("_"))


def generate_rs(generator_arguments_file, typesupport_impls):
    args = read_generator_arguments(generator_arguments_file)
    typesupport_impls = typesupport_impls.split(";")

    template_dir = args["template_dir"]
    type_support_impl_by_filename = {
        "%s_rs.ep.{0}.c".format(impl): impl for impl in typesupport_impls
    }
    mapping_msgs = {
        os.path.join(template_dir, "msg.rs.em"): ["rust/src/%s.rs"],
        os.path.join(
            template_dir, "msg.c.em"
        ): type_support_impl_by_filename.keys(),
    }

    mapping_srvs = {
        os.path.join(template_dir, "srv.rs.em"): ["rust/src/%s.rs"],
        os.path.join(
            template_dir, "srv.c.em"
        ): type_support_impl_by_filename.keys(),
    }

    for template_file in mapping_msgs.keys():
        assert os.path.exists(template_file), (
            "Messages template file %s not found" % template_file
        )
    for template_file in mapping_srvs.keys():
        assert os.path.exists(template_file), (
            "Services template file %s not found" % template_file
        )

    data = {
        "get_c_type": get_c_type,
        "get_builtin_c_type": get_builtin_c_type,
        "get_rs_type": get_rs_type,
        "get_ffi_type": get_ffi_type,
        "get_ffi_return_type": get_ffi_return_type,
        "get_normalized_type": get_normalized_type,
        "constant_value_to_rs": constant_value_to_rs,
        "get_non_array_rs_type": get_non_array_rs_type,
        "value_to_rs": value_to_rs,
        "sanitize_identifier": sanitize_identifier,
        "convert_camel_case_to_lower_case_underscore": convert_camel_case_to_lower_case_underscore,
        "convert_lower_case_underscore_to_camel_case": convert_lower_case_underscore_to_camel_case,
        "get_builtin_rs_type": get_builtin_rs_type,
        "msg_specs": [],
        "srv_specs": [],
        "package_name": args["package_name"],
        "typesupport_impls": typesupport_impls,
        "is_array": is_array,
        "is_string_array": is_string_array,
        "is_primitive_array": is_primitive_array,
        "is_nested_array": is_nested_array,
        "is_static_array": is_static_array,
        "is_dynamic_array": is_dynamic_array,
        "is_static_string_array": is_static_string_array,
        "is_static_primitive_array": is_static_primitive_array,
        "is_static_nested_array": is_static_nested_array,
        "is_dynamic_string_array": is_dynamic_string_array,
        "is_dynamic_primitive_array": is_dynamic_primitive_array,
        "is_dynamic_nested_array": is_dynamic_nested_array,
        "is_dynamic_primitive_array": is_dynamic_primitive_array,
        "is_single_string": is_single_string,
        "is_single_primitive": is_single_primitive,
        "is_single_nested": is_single_nested,
        "is_string": is_string,
        "is_primitive": is_primitive,
        "is_nested": is_nested,
    }
    latest_target_timestamp = get_newest_modification_time(
        args["target_dependencies"]
    )

    for ros_interface_file in args["ros_interface_files"]:
        extension = os.path.splitext(ros_interface_file)[1]
        subfolder = os.path.basename(os.path.dirname(ros_interface_file))
        if extension == ".msg":
            data["msg_specs"].append(
                (
                    subfolder,
                    parse_message_file(
                        args["package_name"], ros_interface_file
                    ),
                )
            )
        elif extension == ".srv":
            data["srv_specs"].append(
                (
                    subfolder,
                    parse_service_file(
                        args["package_name"], ros_interface_file
                    ),
                )
            )
        else:
            continue

    if data["msg_specs"]:
        for template_file, generated_filenames in mapping_msgs.items():
            for generated_filename in generated_filenames:
                generated_file = os.path.join(
                    args["output_dir"], generated_filename % "msg"
                )
                expand_template(
                    os.path.join(template_dir, template_file),
                    data.copy(),
                    generated_file,
                    minimum_timestamp=latest_target_timestamp,
                )

    if data["srv_specs"]:
        for template_file, generated_filenames in mapping_srvs.items():
            for generated_filename in generated_filenames:
                generated_file = os.path.join(
                    args["output_dir"], generated_filename % "srv"
                )
                expand_template(
                    os.path.join(template_dir, template_file),
                    data.copy(),
                    generated_file,
                    minimum_timestamp=latest_target_timestamp,
                )

    expand_template(
        os.path.join(template_dir, "lib.rs.em"),
        data.copy(),
        os.path.join(args["output_dir"], "rust/src/lib.rs"),
        minimum_timestamp=latest_target_timestamp,
    )

    return 0


def escape_string(s):
    s = s.replace("\\", "\\\\")
    s = s.replace("'", "\\'")
    return s


def value_to_rs(type_, value):
    assert type_.is_primitive_type()
    assert value is not None

    if not type_.is_array:
        return primitive_value_to_rs(type_, value)

    rs_values = []
    for single_value in value:
        rs_value = primitive_value_to_rs(type_, single_value)
        rs_values.append(rs_value)
    return "{%s}" % ", ".join(rs_values)


def primitive_value_to_rs(type_, value):
    assert type_.is_primitive_type()
    assert value is not None

    if type_.type == "bool":
        return "true" if value else "false"

    if type_.type in [
        "byte",
        "char",
        "int8",
        "uint8",
        "int16",
        "uint16",
        "int32",
        "uint32",
        "int64",
        "uint64",
        "float64",
    ]:
        return str(value)

    if type_.type == "float32":
        return "%sf" % value

    if type_.type == "string":
        return '"%s"' % escape_string(value)

    assert False, "unknown primitive type '%s'" % type_


def constant_value_to_rs(type_, value):
    assert value is not None

    if type_ == "bool":
        return "true" if value else "false"

    if type_ in [
        "byte",
        "char",
        "int8",
        "uint8",
        "int16",
        "uint16",
        "int32",
        "uint32",
        "int64",
        "uint64",
        "float64",
    ]:
        return str(value)

    if type_ == "float32":
        return "%sf" % value

    if type_ == "string":
        return '"%s"' % escape_string(value)

    assert False, "unknown constant type '%s'" % type_


def get_builtin_rs_type(type_):
    if type_ == "bool":
        return "bool"

    if type_ == "byte":
        return "u8"

    if type_ == "char":
        return "i8"

    if type_ == "float32":
        return "f32"

    if type_ == "float64":
        return "f64"

    if type_ == "int8":
        return "i8"

    if type_ == "uint8":
        return "u8"

    if type_ == "int16":
        return "i16"

    if type_ == "uint16":
        return "u16"

    if type_ == "int32":
        return "i32"

    if type_ == "uint32":
        return "u32"

    if type_ == "int64":
        return "i64"

    if type_ == "uint64":
        return "u64"

    if type_ == "string":
        return "std::string::String"

    assert False, "unknown type '%s'" % type_


def get_non_array_rs_type(type_, package_name, subfolder="msg"):
    if type_.is_primitive_type():
        return get_builtin_rs_type(type_.type)
    else:
        if type_.pkg_name == package_name:
            return "self::%s" % type_.type
        else:
            return "%s::%s::%s" % (type_.pkg_name, subfolder, type_.type)


def get_rs_type(type_, package_name, subfolder="msg"):
    if type_.is_array:
        if type_.is_fixed_size_array():
            return "[%s; %s]" % (
                get_non_array_rs_type(type_, package_name, subfolder),
                type_.array_size,
            )
        else:
            return "Vec<%s>" % get_non_array_rs_type(
                type_, package_name, subfolder
            )
    else:
        return get_non_array_rs_type(type_, package_name, subfolder)


def get_normalized_type(type_, subfolder="msg"):
    if not type_.is_primitive_type():
        return "%s__%s__%s" % (type_.pkg_name, subfolder, type_.type)
    else:
        return get_builtin_rs_type(type_.type)


MSG_TYPE_TO_C = {
    "bool": "bool",
    "byte": "uint8_t",
    "char": "c_char",
    "float32": "c_float",
    "float64": "c_double",
    "uint8": "uint8_t",
    "int8": "int8_t",
    "uint16": "uint16_t",
    "int16": "int16_t",
    "uint32": "uint32_t",
    "int32": "int32_t",
    "uint64": "uint64_t",
    "int64": "int64_t",
    "string": "*const c_char",
}


def get_ffi_type(field, package_name, subfolder="msg"):
    if field.type.is_array and field.type.is_primitive_type():
        return "*const %s" % MSG_TYPE_TO_C[field.type.type]
    elif field.type.is_array:
        return "*const uintptr_t"
    elif field.type.is_primitive_type():
        return MSG_TYPE_TO_C[field.type.type]
    else:
        return "uintptr_t"


def get_ffi_return_type(field, package_name, subfolder="msg"):
    if field.type.is_array and field.type.is_primitive_type():
        return "*const %s" % MSG_TYPE_TO_C[field.type.type]
    elif field.type.is_array:
        return "*const uintptr_t"
    elif field.type.is_primitive_type():
        return MSG_TYPE_TO_C[field.type.type]
    else:
        return "uintptr_t"


def get_builtin_c_type(type_):
    if type_ == "bool":
        return "bool"
    if type_ == "byte":
        return "uint8_t"
    if type_ == "char":
        return "char"
    if type_ == "float32":
        return "float"
    if type_ == "float64":
        return "double"
    if type_ == "int8":
        return "int8_t"
    if type_ == "uint8":
        return "uint8_t"
    if type_ == "int16":
        return "int16_t"
    if type_ == "uint16":
        return "uint16_t"
    if type_ == "int32":
        return "int32_t"
    if type_ == "uint32":
        return "uint32_t"
    if type_ == "int64":
        return "int64_t"
    if type_ == "uint64":
        return "uint64_t"
    if type_ == "string":
        return "const char *"

    assert False, "unknown type '%s'" % type_


def get_c_type(field, subfolder="msg"):
    if field.type.is_array and field.type.is_primitive_type():
        return "%s const*" % get_builtin_c_type(field.type.type)
    elif field.type.is_array:
        return "uintptr_t const*"
    elif field.type.is_primitive_type():
        return get_builtin_c_type(field.type.type)
    else:
        return "uintptr_t"


def sanitize_identifier(identifier):
    rust_exception_keywords = ["crate", "extern", "self", "super", "Self"]
    rust_keywords = [
        "as",
        "break",
        "const",
        "continue",
        "crate",
        "else",
        "enum",
        "extern",
        "false",
        "fn",
        "for",
        "if",
        "impl",
        "in",
        "let",
        "loop",
        "match",
        "mod",
        "move",
        "mut",
        "pub",
        "ref",
        "return",
        "self",
        "Self",
        "static",
        "struct",
        "super",
        "trait",
        "true",
        "type",
        "unsafe",
        "use",
        "where",
        "while",
        "dyn",
        "abstract",
        "become",
        "box",
        "do",
        "final",
        "macro",
        "override",
        "priv",
        "typeof",
        "unsized",
        "virtual",
        "yield",
        "async",
        "await",
        "try",
    ]

    if identifier in rust_exception_keywords:
        return identifier + "_"
    elif identifier in rust_keywords:
        return "r#" + identifier
    else:
        return identifier


def is_array(field):
    return field.type.is_array


def is_nested_array(field):
    return field.type.is_array and not field.type.is_primitive_type()


def is_dynamic_array(field):
    return field.type.is_array and not field.type.is_fixed_size_array()


def is_static_array(field):
    return field.type.is_array and field.type.is_fixed_size_array()


def is_static_string_array(field):
    return (
        field.type.is_array
        and field.type.is_fixed_size_array()
        and field.type.is_primitive_type()
        and field.type.type == "string"
    )


def is_static_primitive_array(field):
    return (
        field.type.is_array
        and field.type.is_fixed_size_array()
        and field.type.is_primitive_type()
    )


def is_static_nested_array(field):
    return (
        field.type.is_array
        and field.type.is_fixed_size_array()
        and not field.type.is_primitive_type()
    )


def is_dynamic_string_array(field):
    return (
        field.type.is_array
        and not field.type.is_fixed_size_array()
        and field.type.is_primitive_type()
        and field.type.type == "string"
    )


def is_dynamic_primitive_array(field):
    return (
        field.type.is_array
        and not field.type.is_fixed_size_array()
        and field.type.is_primitive_type()
        and field.type.type != "string"
    )


def is_dynamic_nested_array(field):
    return (
        field.type.is_array
        and not field.type.is_fixed_size_array()
        and not field.type.is_primitive_type()
    )


def is_string_array(field):
    return (
        field.type.is_array
        and field.type.is_primitive_type()
        and field.type.type == "string"
    )


def is_primitive_array(field):
    return (
        field.type.is_array
        and field.type.is_primitive_type()
        and field.type.type != "string"
    )


def is_single_string(field):
    return (
        not field.type.is_array
        and field.type.is_primitive_type()
        and field.type.type == "string"
    )


def is_single_primitive(field):
    return (
        not field.type.is_array
        and field.type.is_primitive_type()
        and field.type.type != "string"
    )


def is_single_nested(field):
    return not field.type.is_array and not field.type.is_primitive_type()


def is_string(field):
    return field.type.is_primitive_type() and field.type.type == "string"


def is_primitive(field):
    return field.type.is_primitive_type() and field.type.type != "string"


def is_nested(field):
    return not field.type.is_primitive_type()

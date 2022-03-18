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
import pathlib
import subprocess

from pathlib import Path

from rosidl_cmake import convert_camel_case_to_lower_case_underscore
from rosidl_cmake import expand_template
from rosidl_cmake import generate_files
from rosidl_cmake import get_newest_modification_time
from rosidl_cmake import read_generator_arguments

from rosidl_parser.definition import AbstractGenericString
from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import AbstractSequence
from rosidl_parser.definition import AbstractString
from rosidl_parser.definition import AbstractWString
from rosidl_parser.definition import Array
from rosidl_parser.definition import BASIC_TYPES
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import BoundedSequence
from rosidl_parser.definition import BoundedString
from rosidl_parser.definition import BoundedWString
from rosidl_parser.definition import IdlContent
from rosidl_parser.definition import IdlLocator
from rosidl_parser.definition import Message
from rosidl_parser.definition import NamespacedType
from rosidl_parser.definition import Service
from rosidl_parser.definition import UnboundedSequence
from rosidl_parser.definition import UnboundedString
from rosidl_parser.definition import UnboundedWString

from rosidl_parser.parser import parse_idl_file


# Taken from http://stackoverflow.com/a/6425628
def convert_lower_case_underscore_to_camel_case(word):
    return ''.join(x.capitalize() or '_' for x in word.split('_'))


def generate_rs(generator_arguments_file, typesupport_impls):
    args = read_generator_arguments(generator_arguments_file)
    package_name = args['package_name']

    # expand init modules for each directory
    modules = {}
    idl_content = IdlContent()
    (Path(args['output_dir']) / 'rust/src/bindings').mkdir(parents=True, exist_ok=True)
    for idl_tuple in args.get('idl_tuples', []):
        idl_parts = idl_tuple.rsplit(':', 1)
        assert len(idl_parts) == 2

        idl_rel_path = pathlib.Path(idl_parts[1])
        idl_stems = modules.setdefault(str(idl_rel_path.parent), set())
        idl_stems.add(idl_rel_path.stem)

        locator = IdlLocator(*idl_parts)
        idl_file = parse_idl_file(locator)
        idl_content.elements += idl_file.content.elements

    typesupport_impls = typesupport_impls.split(';')

    template_dir = args['template_dir']

    mapping_msgs = {
        os.path.join(template_dir, 'msg.rs.em'): ['rust/src/%s.rs'],
    }

    mapping_srvs = {
        os.path.join(template_dir, 'srv.rs.em'): ['rust/src/%s.rs'],
    }

    # Ensure the required templates exist
    for template_file in mapping_msgs.keys():
        assert os.path.exists(template_file), \
            'Messages template file %s not found' % template_file
    for template_file in mapping_srvs.keys():
        assert os.path.exists(template_file), \
            'Services template file %s not found' % template_file

    data = {
        'get_rmw_rs_type': make_get_rmw_rs_type(args['package_name']),
        'get_rs_name': get_rs_name,
        'get_idiomatic_rs_type': make_get_idiomatic_rs_type(args['package_name']),
        'constant_value_to_rs': constant_value_to_rs,
        'value_to_rs': value_to_rs,
        'convert_camel_case_to_lower_case_underscore':
        convert_camel_case_to_lower_case_underscore,
        'convert_lower_case_underscore_to_camel_case':
        convert_lower_case_underscore_to_camel_case,
        'msg_specs': [],
        'srv_specs': [],
        'package_name': args['package_name'],
        'typesupport_impls': typesupport_impls,
    }

    latest_target_timestamp = get_newest_modification_time(
        args['target_dependencies'])

    for message in idl_content.get_elements_of_type(Message):
        data['msg_specs'].append(('msg', message))

    for service in idl_content.get_elements_of_type(Service):
        data['srv_specs'].append(('srv', service))

    if data['msg_specs']:
        for template_file, generated_filenames in mapping_msgs.items():
            for generated_filename in generated_filenames:
                generated_file = os.path.join(args['output_dir'],
                                              generated_filename % 'msg')
                expand_template(
                    os.path.join(template_dir, template_file),
                    data.copy(),
                    generated_file,
                    minimum_timestamp=latest_target_timestamp)

    if data['srv_specs']:
        for template_file, generated_filenames in mapping_srvs.items():
            for generated_filename in generated_filenames:
                generated_file = os.path.join(args['output_dir'],
                                              generated_filename % 'srv')
                expand_template(
                    os.path.join(template_dir, template_file),
                    data.copy(),
                    generated_file,
                    minimum_timestamp=latest_target_timestamp)

    expand_template(
        os.path.join(template_dir, 'lib.rs.em'),
        data.copy(),
        os.path.join(args['output_dir'], 'rust/src/lib.rs'),
        minimum_timestamp=latest_target_timestamp)

    return 0

def get_rs_name(name):
    keywords = [
        # strict keywords
        'as', 'break', 'const', 'continue', 'crate', 'else', 'enum', 'extern', 'false', 'fn', 'for', 'if', 'for',
        'impl', 'in', 'let', 'loop', 'match', 'mod', 'move', 'mut', 'pub', 'ref', 'return', 'self', 'Self', 'static',
        'struct', 'super', 'trait', 'true', 'type', 'unsafe', 'use', 'where', 'while',
        # Edition 2018+
        'async', 'await', 'dyn',
        # Reserved
        'abstract', 'become', 'box', 'do', 'final', 'macro', 'override', 'priv', 'typeof', 'unsized', 'virtual',
        'yield', 'try'
    ]
    # If the field name is a reserved keyword in Rust append an underscore
    return name if not name in keywords else name + '_'

def escape_string(s):
    s = s.replace('\\', '\\\\')
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
    return '{%s}' % ', '.join(rs_values)


def primitive_value_to_rs(type_, value):
    assert type_.is_primitive_type()
    assert value is not None

    if type_.type == 'bool':
        return 'true' if value else 'false'

    if type_.type in [
            'byte',
            'char',
            'int8',
            'uint8',
            'int16',
            'uint16',
            'int32',
            'uint32',
            'int64',
            'uint64',
            'float64',
    ]:
        return str(value)

    if type_.type == 'float32':
        return '%sf' % value

    if type_.type == 'string':
        return '"%s"' % escape_string(value)

    assert False, "unknown primitive type '%s'" % type_


def constant_value_to_rs(type_, value):
    assert value is not None

    if type_ == 'bool':
        return 'true' if value else 'false'

    if type_ in [
            'byte',
            'char',
            'int8',
            'uint8',
            'int16',
            'uint16',
            'int32',
            'uint32',
            'int64',
            'uint64',
            'float64',
    ]:
        return str(value)

    if type_ == 'float32':
        return '%sf' % value

    if type_ == 'string':
        return '"%s"' % escape_string(value)

    assert False, "unknown constant type '%s'" % type_

# Type hierarchy:
# 
# AbstractType
# - AbstractNestableType
#   - AbstractGenericString
#     - AbstractString
#       - BoundedString
#       - UnboundedString
#     - AbstractWString
#       - BoundedWString
#       - UnboundedWString
#   - BasicType
#   - NamedType
#   - NamespacedType
# - AbstractNestedType
#   - Array
#   - AbstractSequence
#     - BoundedSequence
#     - UnboundedSequence

def make_get_idiomatic_rs_type(package_name):
    get_rmw_rs_type = make_get_rmw_rs_type(package_name)
    def get_idiomatic_rs_type(type_):
        if isinstance(type_, UnboundedString) or isinstance(type_, UnboundedWString):
            return 'std::string::String'
        elif isinstance(type_, UnboundedSequence):
            return 'Vec::<{}>'.format(get_idiomatic_rs_type(type_.value_type))
        elif isinstance(type_, NamespacedType):
            return '::'.join(type_.namespaced_name()).replace(package_name, 'crate')
        elif isinstance(type_, Array):
            return '[{}; {}]'.format(get_idiomatic_rs_type(type_.value_type), type_.size)
        else:
            return get_rmw_rs_type(type_)
    return get_idiomatic_rs_type

def make_get_rmw_rs_type(package_name):
    def get_rmw_rs_type(type_):
        if isinstance(type_, NamespacedType):
            parts = list(type_.namespaced_name())
            parts.insert(-1, 'rmw')
            return '::'.join(parts).replace(package_name, 'crate')
        elif isinstance(type_, BasicType):
            if type_.typename == 'boolean':
                return 'bool'
            elif type_.typename in ['byte', 'octet']:
                return 'u8'
            elif type_.typename == 'char':
                return 'u8'
            elif type_.typename == 'float':
                return 'f32'
            elif type_.typename == 'double':
                return 'f64'
            elif type_.typename == 'int8':
                return 'i8'
            elif type_.typename == 'uint8':
                return 'u8'
            elif type_.typename == 'int16':
                return 'i16'
            elif type_.typename == 'uint16':
                return 'u16'
            elif type_.typename == 'int32':
                return 'i32'
            elif type_.typename == 'uint32':
                return 'u32'
            elif type_.typename == 'int64':
                return 'i64'
            elif type_.typename == 'uint64':
                return 'u64'
        elif isinstance(type_, UnboundedString):
            return 'rosidl_runtime_rs::String'
        elif isinstance(type_, UnboundedWString):
            return 'rosidl_runtime_rs::WString'
        elif isinstance(type_, BoundedString):
            return 'rosidl_runtime_rs::BoundedString<{}>'.format(type_.maximum_size)
        elif isinstance(type_, BoundedWString):
            return 'rosidl_runtime_rs::BoundedWString<{}>'.format(type_.maximum_size)
        elif isinstance(type_, Array):
            return '[{}; {}]'.format(get_rmw_rs_type(type_.value_type), type_.size)
        elif isinstance(type_, UnboundedSequence):
            return 'rosidl_runtime_rs::Sequence<{}>'.format(get_rmw_rs_type(type_.value_type))
        elif isinstance(type_, BoundedSequence):
            return 'rosidl_runtime_rs::BoundedSequence<{}, {}>'.format(get_rmw_rs_type(type_.value_type), type_.maximum_size)

        assert False, "unknown type '%s'" % type_.typename
    return get_rmw_rs_type

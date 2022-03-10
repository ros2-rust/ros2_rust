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

find_package(rmw_implementation_cmake REQUIRED)
find_package(rmw REQUIRED)

if(NOT WIN32)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")
  elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-undefined,error")
  endif()
endif()

# Get a list of typesupport implementations from valid rmw implementations.
rosidl_generator_rs_get_typesupports(_typesupport_impls)

if(_typesupport_impls STREQUAL "")
  message(WARNING "No valid typesupport for Rust generator. Rust messages will not be generated.")
  return()
endif()

set(_output_path
  "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_rs/${PROJECT_NAME}")
set(_generated_extension_files "")
set(_generated_common_rs_files "")

set(_generated_msg_rs_files "")
set(_generated_srv_rs_files "")

set(_has_msg FALSE)
set(_has_srv FALSE)

foreach(_typesupport_impl ${_typesupport_impls})
  set(_generated_extension_${_typesupport_impl}_files "")
endforeach()

foreach(_idl_file ${rosidl_generate_interfaces_ABS_IDL_FILES})
  get_filename_component(_parent_folder "${_idl_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)
  get_filename_component(_module_name "${_idl_file}" NAME_WE)

  if(_parent_folder STREQUAL "msg")
    set(_has_msg TRUE)
    set(_idl_file_without_actions ${_idl_file_without_actions} ${_idl_file})
  elseif(_parent_folder STREQUAL "srv")
    set(_has_srv TRUE)
    set(_idl_file_without_actions ${_idl_file_without_actions} ${_idl_file})
  elseif(_parent_folder STREQUAL "action")
    set(_has_action TRUE)
    message(WARNING "Rust actions generation is not implemented")
  else()
    message(FATAL_ERROR "Interface file with unknown parent folder: ${_idl_file}")
  endif()
endforeach()

list(APPEND _generated_common_rs_files
  "${_output_path}/rust/src/lib.rs"
)

if(${_has_msg})
  list(APPEND _generated_msg_rs_files
    "${_output_path}/rust/src/msg.rs"
  )

  foreach(_typesupport_impl ${_typesupport_impls})
    list_append_unique(_generated_extension_${_typesupport_impl}_files "${_output_path}/msg_rs.ep.${_typesupport_impl}.c")
    list_append_unique(_generated_extension_files "${_generated_extension_${_typesupport_impl}_files}")
  endforeach()
endif()

if(${_has_srv})
  list(APPEND _generated_srv_rs_files
    "${_output_path}/rust/src/srv.rs"
  )

  foreach(_typesupport_impl ${_typesupport_impls})
    list_append_unique(_generated_extension_${_typesupport_impl}_files "${_output_path}/srv_rs.ep.${_typesupport_impl}.c")
    list_append_unique(_generated_extension_files "${_generated_extension_${_typesupport_impl}_files}")
  endforeach()
endif()

set(_dependency_files "")
set(_dependencies "")
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  foreach(_idl_file ${${_pkg_name}_IDL_FILES})
    set(_abs_idl_file "${${_pkg_name}_DIR}/../${_idl_file}")
    normalize_path(_abs_idl_file "${_abs_idl_file}")
    list(APPEND _dependency_files "${_abs_idl_file}")
    list(APPEND _dependencies "${_pkg_name}:${_abs_idl_file}")
  endforeach()
endforeach()

set(target_dependencies
  "${rosidl_generator_rs_BIN}"
  ${rosidl_generator_rs_GENERATOR_FILES}
  "${rosidl_generator_rs_TEMPLATE_DIR}/msg.rs.em"
  "${rosidl_generator_rs_TEMPLATE_DIR}/srv.rs.em"
  ${rosidl_generate_interfaces_ABS_IDL_FILES}
  ${_idl_file_without_actions}
  ${_dependency_files})
foreach(dep ${target_dependencies})
  if(NOT EXISTS "${dep}")
    message(FATAL_ERROR "Target dependency '${dep}' does not exist")
  endif()
endforeach()

set(generator_arguments_file "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_rs__arguments.json")
rosidl_write_generator_arguments(
  "${generator_arguments_file}"
  PACKAGE_NAME "${PROJECT_NAME}"
  IDL_TUPLES "${rosidl_generate_interfaces_IDL_TUPLES}"
  ROS_INTERFACE_FILES "${_idl_file_without_actions}"
  ROS_INTERFACE_DEPENDENCIES "${_dependencies}"
  OUTPUT_DIR "${_output_path}"
  TEMPLATE_DIR "${rosidl_generator_rs_TEMPLATE_DIR}"
  TARGET_DEPENDENCIES ${target_dependencies}
)

file(MAKE_DIRECTORY "${_output_path}")

set(_target_suffix "__rs")

set(CRATES_DEPENDENCIES "rosidl_runtime_rs = \"*\"")
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  find_package(${_pkg_name} REQUIRED)
  set(CRATES_DEPENDENCIES "${CRATES_DEPENDENCIES}\n${_pkg_name} = \"*\"")
endforeach()
ament_index_register_resource("rust_packages")


# needed to avoid multiple calls to the Rust generator trick copied from
# https://github.com/ros2/rosidl/blob/master/rosidl_generator_py/cmake/rosidl_generator_py_generate_interfaces.cmake
set(_subdir "${CMAKE_CURRENT_BINARY_DIR}/${rosidl_generate_interfaces_TARGET}${_target_suffix}")
file(MAKE_DIRECTORY "${_subdir}")
file(READ "${rosidl_generator_rs_DIR}/custom_command.cmake" _custom_command)
file(WRITE "${_subdir}/CMakeLists.txt" "${_custom_command}")
add_subdirectory("${_subdir}" ${rosidl_generate_interfaces_TARGET}${_target_suffix})

add_dependencies(${rosidl_generate_interfaces_TARGET} ${rosidl_generate_interfaces_TARGET}${_target_suffix})

set_property(
  SOURCE
  ${_generated_extension_files}
  ${_generated_common_rs_files}
  ${_generated_msg_rs_files}
  ${_generated_srv_rs_files}
  PROPERTY GENERATED 1)

set(_rsext_suffix "__rsext")
foreach(_typesupport_impl ${_typesupport_impls})
  find_package(${_typesupport_impl} REQUIRED)

  if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
    install(
      DIRECTORY "${_output_path}/rust"
      DESTINATION "share/${PROJECT_NAME}"
    )
  endif()
endforeach()

configure_file("${rosidl_generator_rs_TEMPLATE_DIR}/Cargo.toml.in"
  "share/${PROJECT_NAME}/rust/Cargo.toml"
  @ONLY)

install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/rust/Cargo.toml"
  DESTINATION share/${PROJECT_NAME}/rust/
)

if(BUILD_TESTING AND rosidl_generate_interfaces_ADD_LINTER_TESTS)
  if(
    NOT _generated_msg_rs_files STREQUAL "" OR
    NOT _generated_srv_rs_files STREQUAL ""
  )
  # TODO(esteve): add linters for Rust files
  endif()
endif()

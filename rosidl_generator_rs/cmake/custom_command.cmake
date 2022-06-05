# Copyright 2017 Esteve Fernandez <esteve@apache.org>
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

add_custom_command(
  OUTPUT
  ${_generated_extension_files}
  ${_generated_common_rs_files}
  ${_generated_msg_rs_files}
  ${_generated_msg_c_files}
  ${_generated_srv_rs_files}
  ${_generated_srv_c_files}
  COMMAND ${PYTHON_EXECUTABLE} ${rosidl_generator_rs_BIN}
  --generator-arguments-file "${generator_arguments_file}"
  --typesupport-impls "${_typesupport_impls}"
  DEPENDS ${target_dependencies}
  COMMENT "Generating Rust code for ROS interfaces"
  VERBATIM
)

if(TARGET ${rosidl_generate_interfaces_TARGET}${_target_suffix})
  message(WARNING "Custom target ${rosidl_generate_interfaces_TARGET}${_target_suffix} already exists")
else()
  add_custom_target(
    ${rosidl_generate_interfaces_TARGET}${_target_suffix} ALL
    DEPENDS
    ${_generated_extension_files}
    ${_generated_common_rs_files}
    ${_generated_msg_rs_files}
    ${_generated_msg_c_files}
    ${_generated_srv_rs_files}
    ${_generated_srv_c_files}
  )
endif()

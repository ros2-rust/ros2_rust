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

macro(rosidl_generator_rs_extras BIN GENERATOR_FILES TEMPLATE_DIR)
  find_package(ament_cmake_core QUIET REQUIRED)
  ament_register_extension(
    "rosidl_generate_idl_interfaces"
    "rosidl_generator_rs"
    "rosidl_generator_rs_generate_interfaces.cmake")

  normalize_path(BIN "${BIN}")
  set(rosidl_generator_rs_BIN "${BIN}")

  normalize_path(GENERATOR_FILES "${GENERATOR_FILES}")
  set(rosidl_generator_rs_GENERATOR_FILES "${GENERATOR_FILES}")

  normalize_path(TEMPLATE_DIR "${TEMPLATE_DIR}")
  set(rosidl_generator_rs_TEMPLATE_DIR "${TEMPLATE_DIR}")
endmacro()

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

# list relative paths before absolute paths
set(_AMENT_CMAKE_EXPORT_CRATES
  ${_AMENT_EXPORT_RELATIVE_CRATES}
  ${_AMENT_EXPORT_ABSOLUTE_CRATES})

# generate and register extra file for crates
set(_generated_extra_file
  "${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_export_crates/ament_cmake_export_crates-extras.cmake")
configure_file(
  "${ament_cmake_export_crates_DIR}/ament_cmake_export_crates-extras.cmake.in"
  "${_generated_extra_file}"
  @ONLY
)
list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS "${_generated_extra_file}")

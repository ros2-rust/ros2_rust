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

# copied from
# ament_cmake_export_crates/ament_cmake_export_crates-extras.cmake

# register ament_package() hook for crates once
macro(_ament_cmake_export_crates_register_package_hook)
  if(NOT DEFINED
      _AMENT_CMAKE_EXPORT_CRATES_PACKAGE_HOOK_REGISTERED)
    set(_AMENT_CMAKE_EXPORT_CRATES_PACKAGE_HOOK_REGISTERED TRUE)

    find_package(ament_cmake_core QUIET REQUIRED)
    ament_register_extension("ament_package"
      "ament_cmake_export_crates"
      "ament_cmake_export_crates_package_hook.cmake")
  endif()
endmacro()

include(
  "${ament_cmake_export_crates_DIR}/ament_export_crates.cmake")

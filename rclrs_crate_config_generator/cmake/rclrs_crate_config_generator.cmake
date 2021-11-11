# Copyright 2020 DCS Corporation, All Rights Reserved.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# DISTRIBUTION A. Approved for public release; distribution unlimited.
# OPSEC #4584.

function(rclrs_gen_crate_config ros_crate_list)
  if(NOT ros_crate_list) 
    message(FATAL_ERROR "rclrs_gen_crate_config() requires a non-empty list of ROS 2 Rust crates!")
  endif()
  set(_native_libraries_dirs "")
  set(_found_dirs "")
  set(_crates_dependencies "")
  set(_found_dependencies "")
  set(_found_dependency "")
  set(_ros_crate "")

  message("Input crate list: ${ros_crate_list}")
  foreach(_ros_crate ${ros_crate_list})
    # Get native libraries required for each ROS-provided crate
    foreach(_native_library ${${_ros_crate}_LIBRARIES})
      get_filename_component(_native_library_dir ${_native_library} DIRECTORY)
      message("<CRATE: ${_ros_crate}> Found _native_library_dir: \"${_native_library_dir}\"")
      if((NOT "${_native_library_dir}" STREQUAL "") AND (NOT _native_library_dir IN_LIST _found_dirs))
        message("Appending")
        list(APPEND _found_dirs ${_native_library_dir})
        set(_native_libraries_dirs "${_native_libraries_dirs}\n  '-L native=${_native_library_dir}',")
      endif()
    endforeach()

    # Get crate dependencies for each ROS-provided crate
    foreach(_crate_dependency ${${_ros_crate}_CRATES})
        if(NOT _crate_dependency IN_LIST _found_dependencies)
            list(APPEND _found_dependencies ${_crate_dependency})
            set(_crates_dependencies "${_crates_dependencies}\n${_ros_crate} = {path = '${_crate_dependency}'}")
        endif()
    endforeach()
  endforeach()

  # Set up cargo configuration
  set(build_section "")
  if(NOT "${_native_libraries_dirs}" STREQUAL "")
    set(build_section "[build]\nrustflags = [\n${_native_libraries_dirs}\n]\n")
  endif()
  set(patch_section "")
  if(NOT "${_crates_dependencies}" STREQUAL "")
    set(patch_section "[patch.crates-io]\n${_crates_dependencies}\n")
  endif()

  set(config_output 
    "# This file is generated automatically, and should not be changed by hand\n${build_section}${patch_section}")

  file(MAKE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/.cargo")
  file(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/.cargo/config.toml" ${config_output})
endfunction()
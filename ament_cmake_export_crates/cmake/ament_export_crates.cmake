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

#
# Export CRATE files to downstream packages.
#
# :param ARGN: a list of CRATE files.
#   Each element might either be an absolute path to a CRATE file.
# :type ARGN: list of strings
#
# @public
#
macro(ament_export_crates)
  if(_${PROJECT_NAME}_AMENT_PACKAGE)
    message(FATAL_ERROR
      "ament_export_crates() must be called before ament_package()")
  endif()

  if(${ARGC} GREATER 0)
    _ament_cmake_export_crates_register_package_hook()

    foreach(_arg ${ARGN})
      if(IS_ABSOLUTE "${_arg}")
        if(NOT EXISTS "${_arg}")
          message(WARNING
            "ament_export_crates() package '${PROJECT_NAME}' exports the "
            "crate '${_arg}' which doesn't exist")
        else()
            list_append_unique(_AMENT_EXPORT_ABSOLUTE_CRATES "${_arg}")
        endif()
      else()
        set(_arg "\${${PROJECT_NAME}_DIR}/../../../${_arg}")
        list_append_unique(_AMENT_EXPORT_RELATIVE_CRATES "${_arg}")
      endif()
    endforeach()
  endif()
endmacro()

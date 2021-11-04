cmake_minimum_required(VERSION 3.5)
project(rclrs_examples NONE)

find_package(ament_cmake REQUIRED)
find_package(rclrs_common REQUIRED)
find_package(rclrs REQUIRED)
find_package(std_msgs REQUIRED)

set(_native_libraries_dirs "")
set(_found_dirs "")
foreach(_native_library ${std_msgs_LIBRARIES})
    get_filename_component(_native_library_dir ${_native_library} DIRECTORY)
    if((NOT _native_library_dir STREQUAL "") AND (NOT _native_library_dir IN_LIST _found_dirs))
        list(APPEND _found_dirs ${_native_library_dir})
        set(_native_libraries_dirs "${_native_libraries_dirs}\n'-L native=${_native_library_dir}',")
    endif()
endforeach()

set(_crates_dependencies "")
set(_found_dependencies "")
foreach(_crate_dependency ${std_msgs_CRATES})
    if(NOT _crate_dependency IN_LIST _found_dependencies)
        list(APPEND _found_dependencies ${_crate_dependency})
        set(_crates_dependencies "${_crates_dependencies}\nstd_msgs = {path = '${_crate_dependency}'}\n")
    endif()
endforeach()
include(ExternalProject)

# Set up cargo configuration
file(MAKE_DIRECTORY "${CMAKE_SOURCE_DIR}/.cargo")
file(WRITE "${CMAKE_SOURCE_DIR}/.cargo/config.toml"
"#This file is generated automatically, and should not be changed by hand\n\
[build]\n\
rustflags = [\
${_native_libraries_dirs}
]\n\n\
[patch.crates-io]\
${_crates_dependencies}\
"
)

# Build examples
add_custom_target(
    build_crate ALL
    COMMAND cargo build $<$<NOT:$<CONFIG:Debug>>:--release>
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

install(FILES
    ${CMAKE_SOURCE_DIR}/target/$<$<NOT:$<CONFIG:Debug>>:release>$<$<CONFIG:Debug>:debug>/rclrs_publisher
    ${CMAKE_SOURCE_DIR}/target/$<$<NOT:$<CONFIG:Debug>>:release>$<$<CONFIG:Debug>:debug>/rclrs_subscriber
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()

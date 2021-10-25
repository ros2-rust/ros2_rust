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
        set(_crates_dependencies "${_crates_dependencies}\n[dependencies.std_msgs]\npath = '${_crate_dependency}'\n")
    endif()
endforeach()

include(ExternalProject)

configure_file(${CMAKE_SOURCE_DIR}/src/rclrs_publisher.rs ${CMAKE_BINARY_DIR}/src/rclrs_publisher.rs COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/src/rclrs_subscriber.rs ${CMAKE_BINARY_DIR}/src/rclrs_subscriber.rs COPYONLY)

file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/.cargo")
file(WRITE "${CMAKE_BINARY_DIR}/.cargo/config"
"\
[build]\n\
target-dir = 'ament_cargo/${PROJECT_NAME}/target'\n\
rustflags = [\
${_native_libraries_dirs}
]\n\
"
)

file(COPY "${CMAKE_SOURCE_DIR}/Cargo.toml" DESTINATION "${CMAKE_BINARY_DIR}/")
file(APPEND "${CMAKE_BINARY_DIR}/Cargo.toml" "${_crates_dependencies}")

add_custom_command(
    OUTPUT
        ${CMAKE_BINARY_DIR}/ament_cargo/${PROJECT_NAME}/target/release/rclrs_publisher
        ${CMAKE_BINARY_DIR}/ament_cargo/${PROJECT_NAME}/target/release/rclrs_subscriber
    COMMAND cargo build --release --manifest-path "${CMAKE_BINARY_DIR}/Cargo.toml"
    DEPENDS
        ${CMAKE_BINARY_DIR}/src/rclrs_publisher.rs
        ${CMAKE_BINARY_DIR}/src/rclrs_subscriber.rs
)

add_custom_target(
    build_crate ALL
    DEPENDS
        ${CMAKE_BINARY_DIR}/ament_cargo/${PROJECT_NAME}/target/release/rclrs_publisher
        ${CMAKE_BINARY_DIR}/ament_cargo/${PROJECT_NAME}/target/release/rclrs_subscriber
)

install(FILES
    ${CMAKE_BINARY_DIR}/ament_cargo/${PROJECT_NAME}/target/release/rclrs_publisher
    ${CMAKE_BINARY_DIR}/ament_cargo/${PROJECT_NAME}/target/release/rclrs_subscriber
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
    DESTINATION lib/${PROJECT_NAME}
)
ament_package()
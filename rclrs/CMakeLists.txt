cmake_minimum_required(VERSION 3.5)
project(rclrs NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_export_crates REQUIRED)
find_package(rclrs_crate_config_generator REQUIRED)
find_package(rclrs_msg_utilities REQUIRED)

ament_export_crates(${CMAKE_SOURCE_DIR})

set(ROS_CRATES rclrs_msg_utilities)
rclrs_gen_crate_config(${ROS_CRATES})

# Build library
add_custom_target(
    build_crate ALL
    COMMAND cargo build $<$<NOT:$<CONFIG:Debug>>:--release>
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

install(FILES
    ${CMAKE_SOURCE_DIR}/target/$<$<NOT:$<CONFIG:Debug>>:release>$<$<CONFIG:Debug>:debug>/librclrs.rlib
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()

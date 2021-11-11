cmake_minimum_required(VERSION 3.5)
project(rclrs_examples NONE)

find_package(ament_cmake REQUIRED)
find_package(rclrs_crate_config_generator REQUIRED)
find_package(rclrs REQUIRED)
find_package(rclrs_msg_utilities REQUIRED)
find_package(std_msgs REQUIRED)

set(ROS_CRATES std_msgs rclrs rclrs_msg_utilities)
rclrs_gen_crate_config("${ROS_CRATES}")

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

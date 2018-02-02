cmake_minimum_required(VERSION 3.5)

project(rclrs C)

find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_export_crates REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclrs_common REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(rosidl_generator_c REQUIRED)

set(${PROJECT_NAME}_SRCS
  "src/c/rclrs.c"
)

set(CMAKE_C_STANDARD 11)

add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_SRCS})

ament_target_dependencies(${PROJECT_NAME}
  "builtin_interfaces"
  "rcl"
  "rosidl_generator_c"
  "rosidl_typesupport_c")

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

configure_file(
  "src/rust/Cargo.toml.in"
  "share/${PROJECT_NAME}/rust/Cargo.toml"
  @ONLY
)

install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/rust/Cargo.toml"
  DESTINATION share/${PROJECT_NAME}/rust/
)

install(
  DIRECTORY src/rust/src/
  DESTINATION  share/${PROJECT_NAME}/rust/src
)

install(
  DIRECTORY src/rust/src/
  DESTINATION  share/${PROJECT_NAME}/rust/src
)

ament_export_dependencies(ament_cmake_ros)
ament_export_dependencies(builtin_interfaces)
ament_export_dependencies(rcl)
ament_export_dependencies(rosidl_generator_c)
ament_export_dependencies(rosidl_typesupport_c)

ament_export_libraries(${PROJECT_NAME})
ament_export_crates("share/${PROJECT_NAME}/rust/")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

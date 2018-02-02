cmake_minimum_required(VERSION 3.5)

project(rosidl_generator_rs)

find_package(ament_cmake REQUIRED)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rosidl_cmake)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS bin/rosidl_generator_rs
  DESTINATION lib/rosidl_generator_rs
)

install(
  DIRECTORY cmake resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package(
  CONFIG_EXTRAS "cmake/rosidl_generator_rs_get_typesupports.cmake"
    "cmake/register_rs.cmake"
    "rosidl_generator_rs-extras.cmake.in"
)

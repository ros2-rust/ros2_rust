cmake_minimum_required(VERSION 3.5)

project(rosidl_generator_rs)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(rosidl_generator_c REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)
find_package(rosidl_typesupport_introspection_c REQUIRED)

ament_export_dependencies(rosidl_cmake)
ament_export_dependencies(rmw)
ament_export_dependencies(rosidl_generator_c)

ament_index_register_resource("rosidl_generator_packages")
ament_index_register_resource("rosidl_runtime_packages")

ament_python_install_package(${PROJECT_NAME})

ament_package(
  CONFIG_EXTRAS "rosidl_generator_rs-extras.cmake.in"
    "cmake/rosidl_generator_rs_get_typesupports.cmake"
    "cmake/register_rs.cmake"
)

install(DIRECTORY cmake
        DESTINATION share/${PROJECT_NAME})
ament_register_extension(
        "rosidl_generate_idl_interfaces"
        "rosidl_generator_rs"
        "${PROJECT_SOURCE_DIR}/cmake/rosidl_generator_rs_generate_interfaces.cmake")

install(
  PROGRAMS bin/rosidl_generator_rs
  DESTINATION lib/rosidl_generator_rs
)
install(
  DIRECTORY cmake resource
  DESTINATION share/${PROJECT_NAME}
)

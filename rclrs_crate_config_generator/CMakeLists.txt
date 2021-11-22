cmake_minimum_required(VERSION 3.5)

project(rclrs_crate_config_generator NONE)

find_package(ament_cmake_core REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(
  CONFIG_EXTRAS "cmake/rclrs_crate_config_generator.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

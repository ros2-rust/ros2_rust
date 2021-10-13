cmake_minimum_required(VERSION 3.5)
project(rclrs_core NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_export_crates REQUIRED)

install(
  FILES Cargo.toml
  DESTINATION share/${PROJECT_NAME}/rust/
)

install(
  DIRECTORY src/
  DESTINATION  share/${PROJECT_NAME}/rust/src
)

ament_export_crates("share/${PROJECT_NAME}/rust")

ament_package()
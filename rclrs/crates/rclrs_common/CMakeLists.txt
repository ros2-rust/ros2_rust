cmake_minimum_required(VERSION 3.5)
project(rclrs_common NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_export_crates REQUIRED)

message("<rclrs_common> BEGIN CMAKE")
# file(COPY "${CMAKE_SOURCE_DIR}/Cargo.toml" DESTINATION "${CMAKE_BINARY_DIR}/")

message("<rclrs_common> install destination: share/${PROJECT_NAME}/rust/")

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

message("<rclrs_common> CMAKE END")

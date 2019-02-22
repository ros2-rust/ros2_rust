cmake_minimum_required(VERSION 3.5)
project(rclrs NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_export_crates REQUIRED)
find_package(rclrs_common REQUIRED)

foreach(_crate_dependency ${rclrs_common_CRATES})
    set(_crates_dependencies "${_crates_dependencies}\n[dependencies.rclrs_common]\npath = '${_crate_dependency}'\n")
endforeach()

file(COPY "${CMAKE_SOURCE_DIR}/Cargo.toml" DESTINATION "${CMAKE_BINARY_DIR}/")
file(APPEND "${CMAKE_BINARY_DIR}/Cargo.toml" "${_crates_dependencies}")

install(
  FILES ${CMAKE_BINARY_DIR}/Cargo.toml build.rs src/rcl_wrapper.h
  DESTINATION share/${PROJECT_NAME}/rust/
)

install(
  DIRECTORY src/
  DESTINATION  share/${PROJECT_NAME}/rust/src
)

ament_export_crates("share/${PROJECT_NAME}/rust")

ament_package()
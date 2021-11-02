cmake_minimum_required(VERSION 3.5)
project(rclrs_common NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_export_crates REQUIRED)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CARGO_RELEASE_FLAG "--release" CACHE INTERNAL "")
  set(TARGET_DIR "release" CACHE INTERNAL "")
else()
  set(CARGO_RELEASE_FLAG "" CACHE INTERNAL "")
  set(TARGET_DIR "debug" CACHE INTERNAL "")
endif()

set(AMENT_CARGO_TARGET_DIR "${CMAKE_BINARY_DIR}/ament_cargo/${PROJECT_NAME}/target" CACHE INTERNAL "")

message("<rclrs_common> BEGIN CMAKE")

# Copy source to exportable location
file(COPY "${CMAKE_SOURCE_DIR}/Cargo.toml" DESTINATION "${CMAKE_BINARY_DIR}/")
file(COPY "${CMAKE_SOURCE_DIR}/src/" DESTINATION "${CMAKE_BINARY_DIR}/src/")


install(
  FILES Cargo.toml
  DESTINATION share/${PROJECT_NAME}/rust/
)

install(
  DIRECTORY src/
  DESTINATION share/${PROJECT_NAME}/rust/src
)

ament_export_crates("share/${PROJECT_NAME}/rust")

# Build library
add_custom_target(
  build_crate ALL
  COMMAND cargo build ${CARGO_RELEASE_FLAG} --manifest-path ${CMAKE_BINARY_DIR}/Cargo.toml --target-dir ${AMENT_CARGO_TARGET_DIR}
)

install(FILES
  ${AMENT_CARGO_TARGET_DIR}/${TARGET_DIR}/librclrs_common.rlib
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

message("<rclrs_common> CMAKE END")

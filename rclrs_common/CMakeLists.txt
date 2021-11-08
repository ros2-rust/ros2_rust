cmake_minimum_required(VERSION 3.5)
project(rclrs_common NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_export_crates REQUIRED)

ament_export_crates(${CMAKE_SOURCE_DIR})

# Build library
add_custom_target(
  build_crate ALL
  COMMAND cargo build $<$<NOT:$<CONFIG:Debug>>:--release>
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

install(FILES
  target/$<$<NOT:$<CONFIG:Debug>>:release>$<$<CONFIG:Debug>:debug>/librclrs_common.rlib
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
  DESTINATION lib/${PROJECT_NAME}
)

if (BUILD_TESTING)
add_test(NAME rclrs_common_test
  COMMAND cargo test
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()

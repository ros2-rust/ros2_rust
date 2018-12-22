cmake_minimum_required(VERSION 3.5)

project(test_msgs)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

set(msg_files
  "msg/BoundedArrayPrimitivesNested.msg"
  "msg/BoundedArrayNested.msg"
  "msg/BoundedArrayPrimitives.msg"
  "msg/Builtins.msg"
  "msg/DynamicArrayNested.msg"
  "msg/DynamicArrayPrimitives.msg"
  "msg/DynamicArrayPrimitivesNested.msg"
  "msg/DynamicArrayStaticArrayPrimitivesNested.msg"
  "msg/Empty.msg"
  "msg/Nested.msg"
  "msg/Primitives.msg"
  "msg/StaticArrayNested.msg"
  "msg/StaticArrayPrimitives.msg"
  "msg/StaticArrayPrimitivesNested.msg"
)

set(srv_files
  "srv/Empty.srv"
  "srv/Primitives.srv"
)

set(action_files
  "action/Fibonacci.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces action_msgs unique_identifier_msgs
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_action_typesupport_c_builds
    test/test_c_type_support.cpp)
  target_include_directories(test_action_typesupport_c_builds PRIVATE
    include)
  target_link_libraries(test_action_typesupport_c_builds
    ${_AMENT_EXPORT_LIBRARIES})

  ament_add_gtest(test_action_typesupport_cpp_builds
    test/test_cpp_type_support.cpp)
  target_include_directories(test_action_typesupport_cpp_builds PRIVATE
    include)
  target_link_libraries(test_action_typesupport_cpp_builds
    ${_AMENT_EXPORT_LIBRARIES})
endif()

if(DEFINED PYTHON_INSTALL_DIR)
  install(DIRECTORY src/test_msgs
    DESTINATION "${PYTHON_INSTALL_DIR}")
endif()

install(DIRECTORY include/
  DESTINATION include)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_package()

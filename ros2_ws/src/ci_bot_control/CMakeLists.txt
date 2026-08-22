cmake_minimum_required(VERSION 3.8)
project(ci_bot_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(velocity_guard_lib
  src/velocity_guard.cpp
)

target_include_directories(velocity_guard_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  velocity_guard_lib
  rclcpp
  geometry_msgs
)

add_executable(velocity_guard_node
  src/velocity_guard_node.cpp
)

target_link_libraries(
  velocity_guard_node
  velocity_guard_lib
)

ament_target_dependencies(
  velocity_guard_node
  rclcpp
  geometry_msgs
)

install(
  TARGETS
    velocity_guard_lib
    velocity_guard_node
  EXPORT export_velocity_guard
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_velocity_guard
    test/test_velocity_guard.cpp
  )

  target_link_libraries(
    test_velocity_guard
    velocity_guard_lib
  )
endif()

ament_export_targets(export_velocity_guard HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs)

ament_package()

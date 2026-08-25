cmake_minimum_required(VERSION 3.8)
project(ci_bot_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(
    test/test_gazebo_movement.py
    TIMEOUT 40
  )
endif()

ament_package()

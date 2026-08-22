cmake_minimum_required(VERSION 3.8)
project(ci_bot_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

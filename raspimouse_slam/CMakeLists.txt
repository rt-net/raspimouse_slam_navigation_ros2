cmake_minimum_required(VERSION 3.5)
project(raspimouse_slam)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

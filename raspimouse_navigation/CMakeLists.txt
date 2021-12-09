cmake_minimum_required(VERSION 3.5)
project(raspimouse_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
    launch params rviz
    DESTINATION share/${PROJECT_NAME}
)

ament_package()

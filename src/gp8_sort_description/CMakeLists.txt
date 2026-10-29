cmake_minimum_required(VERSION 3.8)
project(gp8_sort_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()

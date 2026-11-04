cmake_minimum_required(VERSION 3.8)
project(slam_robot)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch config worlds rviz maps scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

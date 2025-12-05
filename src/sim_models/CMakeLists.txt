cmake_minimum_required(VERSION 3.8)
project(sim_models)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch models worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.5)
project(driver_bringup)

find_package(ament_cmake REQUIRED)

# Install config directory
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

# Install launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()

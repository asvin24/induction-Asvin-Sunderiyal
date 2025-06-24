cmake_minimum_required(VERSION 3.5)
project(turtle_controller)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python script
install(
  PROGRAMS
    scripts/draw_circle.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python module
install(
  DIRECTORY turtle_controller
  DESTINATION lib/python3.10/site-packages
)

ament_package()


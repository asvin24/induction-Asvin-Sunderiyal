
cmake_minimum_required(VERSION 3.5)
project(obstacle_avoider)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Install Python script
install(
  PROGRAMS
    scripts/obstacle_avoider.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

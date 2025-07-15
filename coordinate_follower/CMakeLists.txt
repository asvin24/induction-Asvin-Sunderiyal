cmake_minimum_required(VERSION 3.8)
project(coordinate_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToPos.action"
  DEPENDENCIES geometry_msgs nav_msgs
)

install(PROGRAMS
  scripts/move_to_pos_server.py
  scripts/move_to_pos_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  rosidl_default_runtime
  geometry_msgs
  nav_msgs
  action_msgs
)

ament_package()

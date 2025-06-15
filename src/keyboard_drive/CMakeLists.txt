cmake_minimum_required(VERSION 3.5)
project(keyboard_drive)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(CUSTOM_NODE keyboard_drive_node)

add_executable(${CUSTOM_NODE}
  src/${CUSTOM_NODE}.cpp
)

ament_target_dependencies(${CUSTOM_NODE}
  rclcpp
  std_msgs
  geometry_msgs
)

# Install the launch directory
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${CUSTOM_NODE}
  DESTINATION lib/${PROJECT_NAME}
)

# Export package
ament_package()

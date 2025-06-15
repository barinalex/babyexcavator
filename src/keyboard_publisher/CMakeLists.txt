cmake_minimum_required(VERSION 3.5)
project(keyboard_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(keyboard_handler REQUIRED)
find_package(std_msgs REQUIRED)

set(KEYBOARD_PUBLISHER_NODE keyboard_publisher_node)

add_executable(${KEYBOARD_PUBLISHER_NODE}
  src/${KEYBOARD_PUBLISHER_NODE}.cpp
)

ament_target_dependencies(${KEYBOARD_PUBLISHER_NODE}
  rclcpp
  std_msgs
  keyboard_handler
)

install(TARGETS
  ${KEYBOARD_PUBLISHER_NODE}
  DESTINATION lib/${PROJECT_NAME}
)

# Export package
ament_package()

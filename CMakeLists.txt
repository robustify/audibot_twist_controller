cmake_minimum_required(VERSION 3.5)
project(audibot_twist_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/AudibotSpeedNode.cpp
  src/AudibotSteerNode.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  example_interfaces
  geometry_msgs
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<INSTALL_INTERFACE:include>
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "${PROJECT_NAME}::AudibotSpeedNode"
  EXECUTABLE audibot_speed_node
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "${PROJECT_NAME}::AudibotSteerNode"
  EXECUTABLE audibot_steer_node
)

### Install Rules
install(
  TARGETS
    ${PROJECT_NAME}
    audibot_speed_node
    audibot_steer_node
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
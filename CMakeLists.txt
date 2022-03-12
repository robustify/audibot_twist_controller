cmake_minimum_required(VERSION 3.0.2)
project(audibot_twist_controller)

# set default build type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

SET(CMAKE_CPP_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/AudibotSpeed.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/AudibotSpeedNode.cpp
  src/AudibotSteeringNode.cpp
)
add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(audibot_speed_node
  src/audibot_speed_node.cpp
)
target_link_libraries(audibot_speed_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(audibot_steering_node
  src/audibot_steering_node.cpp
)
target_link_libraries(audibot_steering_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(audibot_twist_node
  src/audibot_twist_node.cpp
)
target_link_libraries(audibot_twist_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

### Install Rules
install(
  TARGETS
    ${PROJECT_NAME}
    audibot_speed_node
    audibot_steering_node
    audibot_twist_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
cmake_minimum_required(VERSION 2.8.3)
project(motors_custom_controller)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

message(STATUS ${CMAKE_MODULE_PATH})

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    message_generation
    message_runtime
    roscpp
    rospy
    std_msgs
    controller_interface
    qt_motor)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  StartStop.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


# Declare catkin project
catkin_package(
    CATKIN_DEPENDS controller_interface qt_motor
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    std_msgs
    message_runtime
)

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  ${qt_motor_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  include/motors_custom_controller/motors_custom_controller.h
  src/motors_custom_controller.cpp
)
target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      ${TinyXML_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES motors_custom_controller_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

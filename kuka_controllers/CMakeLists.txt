cmake_minimum_required(VERSION 3.0.2)
project(kuka_controllers)


find_package(catkin REQUIRED COMPONENTS
  controller_interface
  control_msgs
  geometry_msgs
  forward_command_controller
  control_toolbox
  realtime_tools
  urdf
  roscpp
  kdl_parser
  message_generation
  cmake_modules
  kdl_conversions
  sensor_msgs
  std_msgs
  tf_conversions
  pluginlib
  orocos_kdl
)

add_definitions (-fpermissive -std=c++11)

find_package(orocos_kdl REQUIRED)
include_directories(${orocos_kdl_INCLUDE_DIRS})


include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

find_package(catkin REQUIRED message_generation std_msgs)

add_message_files(
  FILES
  KukaState.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS
    controller_interface
    control_msgs
    geometry_msgs
    control_toolbox
    realtime_tools
    urdf
    forward_command_controller
    kdl_parser
    message_runtime
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
  src/kuka_inverse_dynamic_controller.cpp
  src/kuka_kuka_state_controller.cpp
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/include)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg)
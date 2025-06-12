cmake_minimum_required(VERSION 3.0.2)
project(spar_lab_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  moveit_core
  moveit_ros_perception
  moveit_ros_planning_interface
  roscpp
  rospy
)

find_package(Boost REQUIRED system filesystem date_time thread)
find_package(yaml-cpp REQUIRED)

add_service_files(FILES
    jointGoal.srv
    poseGoal.srv
    positionGoal.srv
    waypoint_record.srv
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
    std_msgs
)

set(THIS_PACKAGE_INCLUDE_DIRS
  include
)

catkin_package(
  LIBRARIES
    interactivity_utils
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
)



include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} )
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(control 
  src/control.cpp
)

target_link_libraries(control ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS control DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

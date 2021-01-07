cmake_minimum_required(VERSION 3.0.2)
project(motion_planning)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Setup python modules with setup.py
catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  DMPState.msg
  TrajGenState.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Mark executable scripts (Python etc.) for installation
install(PROGRAMS
     nodes/dmp_node
     nodes/traj_scaling_node
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
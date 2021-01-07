cmake_minimum_required(VERSION 3.0.2)
project(experiment_dmp_tc)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package(
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
     scripts/initialize.py
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

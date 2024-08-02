cmake_minimum_required(VERSION 3.0.2)
project(ros_uwb_dwm1001)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs tf
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Mark scripts for installation
catkin_install_python(PROGRAMS
  scripts/dwm1001c.py
  scripts/transform.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and config files)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)

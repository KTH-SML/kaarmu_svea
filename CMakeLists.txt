cmake_minimum_required(VERSION 3.0.2)
project(rsu_core)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  aruco_msgs
  cv_bridge
  geometry_msgs
  image_geometry
  rospy
  rosonic
  rsu_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf_conversions
  # zed_wrapper # This is necessary to run a ZED camera, you can find it [here](https://github.com/stereolabs/zed-ros-wrapper)
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
  aruco_msgs
  rsu_msgs
  sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

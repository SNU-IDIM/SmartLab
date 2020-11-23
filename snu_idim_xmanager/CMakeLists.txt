cmake_minimum_required(VERSION 2.8.3)
project(snu_idim_xmanager)
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  actionlib_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES snu_idim_xmanager
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
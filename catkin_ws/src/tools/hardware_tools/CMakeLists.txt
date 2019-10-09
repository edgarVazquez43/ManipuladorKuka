cmake_minimum_required(VERSION 2.8.3)
project(hardware_tools)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  #hri_msgs
  #manip_msgs
  #navig_msgs
  geometry_msgs
  roscpp
  rospy
  roslib
  sensor_msgs
  std_msgs
  vision_msgs
  point_cloud_manager
  tf
  tf_conversions
  pcl_conversions
  std_srvs
  message_generation
)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hardware_tools
  CATKIN_DEPENDS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  vision_msgs
  pcl_conversions
  std_srvs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(hardware_tools
  src/VisionTools.cpp
)

add_dependencies(hardware_tools
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(hardware_tools
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

cmake_minimum_required(VERSION 3.0.2)
project(kinect_robot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  message_generation
)

find_package(PCL REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  FILES 
  SegmentedClustersArray.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES kinect_robot
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs geometry_msgs visualization_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (filter
  src/filter.cpp
)
target_link_libraries (filter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable (segmen
  src/segmen.cpp
)
target_link_libraries (segmen
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable (cluster
  src/cluster.cpp
)
target_link_libraries (cluster
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

#add_executable (pcl_process
#  src/filter.cpp
#  src/identify_ground.cpp
#  src/cluster.cpp
#)
#target_link_libraries (pcl_process
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#  ${Boost_LIBRARIES}
#)


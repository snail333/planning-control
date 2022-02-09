cmake_minimum_required(VERSION 3.0.2)
project(rospython)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
)

catkin_package(
	INCLUDE_DIRS
)

include_directories(
	${catkin_INCLUDE_DIRS}
)

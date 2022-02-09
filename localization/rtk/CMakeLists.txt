cmake_minimum_required(VERSION 2.8.3)
project(rtk)

find_package(
  catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    rospy
    std_msgs
    sensor_msgs
    common_msgs
    tf
    tf2
    llh2utm
)

catkin_package()

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  rtk_localization
  src/rtk_localization.cpp
)
target_link_libraries(rtk_localization ${catkin_LIBRARIES})
add_dependencies(rtk_localization ${catkin_EXPORTED_TARGETS})

install(
  TARGETS rtk_localization
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)




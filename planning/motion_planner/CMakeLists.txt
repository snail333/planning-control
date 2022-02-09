cmake_minimum_required(VERSION 2.8.3)
project(motion_planner)

set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(
  catkin REQUIRED COMPONENTS
    common_msgs
    libtrajectory_follower
    pcl_conversions
    pcl_ros
    roscpp
    roslint
    std_msgs
    tf
)

catkin_package()

# TODO add all codes to roslint


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  velocity_set
  src/velocity_set/velocity_set.cpp
  src/velocity_set/velocity_set_path.cpp
  src/velocity_set/velocity_set_info.cpp
)

target_link_libraries(
  velocity_set
  ${catkin_LIBRARIES}
)

add_dependencies(
  velocity_set
  ${catkin_EXPORTED_TARGETS}
)


install(
  TARGETS velocity_set 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

if (CATKIN_ENABLE_TESTING)
  roslint_add_test()
endif()

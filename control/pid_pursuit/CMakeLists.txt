cmake_minimum_required(VERSION 2.8.3)
project(pid_pursuit)

find_package(
  catkin REQUIRED COMPONENTS
    common_msgs
    geometry_msgs
    libtrajectory_follower
    roscpp
    roslint
    rostest
    rosunit
    std_msgs
    tf
    tf2
)

catkin_package(
  INCLUDE_DIRS include
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

set(ROSLINT_CPP_OPTS "--filter=-build/c++11")
roslint_cpp()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  pid_pursuit
  src/pure_pursuit_node.cpp
  src/pure_pursuit.cpp
  src/pure_pursuit_core.cpp
  src/pure_pursuit_viz.cpp
  src/pid_control.cpp
)
target_link_libraries(pid_pursuit ${catkin_LIBRARIES})
add_dependencies(pid_pursuit ${catkin_EXPORTED_TARGETS})

install(
  TARGETS pid_pursuit
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  roslint_add_test()
endif()

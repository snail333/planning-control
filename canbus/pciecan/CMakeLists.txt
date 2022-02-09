cmake_minimum_required(VERSION 2.8.3)
project(pciecan)

find_package(
  catkin REQUIRED COMPONENTS
    common_msgs
    geometry_msgs
    roscpp
    roslint
    rostest
    rosunit
    std_msgs
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
  pciecan
  src/pciecan_node.cpp
  src/pciecan.cpp
)
target_link_libraries(pciecan ${catkin_LIBRARIES})
add_dependencies(pciecan ${catkin_EXPORTED_TARGETS})

install(
  TARGETS pciecan
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

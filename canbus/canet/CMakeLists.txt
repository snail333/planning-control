cmake_minimum_required(VERSION 2.8.3)
project(canet)

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
  canet
  src/canet_node.cpp
  src/canet.cpp
)
target_link_libraries(canet ${catkin_LIBRARIES})
add_dependencies(canet ${catkin_EXPORTED_TARGETS})

install(
  TARGETS canet
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

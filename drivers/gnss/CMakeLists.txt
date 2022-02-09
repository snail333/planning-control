cmake_minimum_required(VERSION 2.8.3)
project(gnss)

find_package(
  catkin REQUIRED COMPONENTS
    roscpp
    rospy
    serial
    roslint
    rostest
    rosunit
    std_msgs
    common_msgs
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
  gnss
  src/gnss_driver.cpp
)
target_link_libraries(gnss ${catkin_LIBRARIES})
add_dependencies(gnss ${catkin_EXPORTED_TARGETS})

install(
  TARGETS gnss
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

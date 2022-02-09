cmake_minimum_required(VERSION 2.8.12)
project(lane_select)

find_package(catkin REQUIRED COMPONENTS
  common_msgs
  libtrajectory_follower
  roscpp
  std_msgs
)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(
  lane_select
  nodes/lane_select/lane_select_node.cpp
  nodes/lane_select/lane_select_core.cpp
)

target_link_libraries(lane_select ${catkin_LIBRARIES})

add_dependencies(
  lane_select
  ${catkin_EXPORTED_TARGETS}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS
    lane_select
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
    add_rostest_gtest(test-lane_select
      test/test_lane_select.test
      test/src/test_lane_select.cpp
      nodes/lane_select/lane_select_core.cpp
    )
    add_dependencies(test-lane_select ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test-lane_select
    ${catkin_LIBRARIES})
endif()

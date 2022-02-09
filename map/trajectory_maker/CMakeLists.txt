cmake_minimum_required(VERSION 2.8.3)
project(trajectory_maker)

find_package(
  catkin REQUIRED COMPONENTS
    common_msgs
    message_filters
    geometry_msgs
    libtrajectory_follower
    nav_msgs
    roscpp
    std_msgs
    tf
)

find_package(Boost REQUIRED)

catkin_package()

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  trajectory_loader
  nodes/trajectory_loader/trajectory_loader_core.cpp
  nodes/trajectory_loader/trajectory_loader_node.cpp
)
target_link_libraries(trajectory_loader ${catkin_LIBRARIES})
add_dependencies(trajectory_loader ${catkin_EXPORTED_TARGETS})

add_executable(
  trajectory_replanner
  nodes/trajectory_replanner/trajectory_replanner.cpp
  nodes/trajectory_replanner/trajectory_replanner_node.cpp
)

target_link_libraries(trajectory_replanner ${catkin_LIBRARIES})
add_dependencies(trajectory_replanner ${catkin_EXPORTED_TARGETS})

add_executable(trajectory_saver nodes/trajectory_saver/trajectory_saver.cpp)
target_link_libraries(trajectory_saver ${catkin_LIBRARIES})
add_dependencies(trajectory_saver ${catkin_EXPORTED_TARGETS})

add_executable(
  trajectory_marker_publisher
  nodes/trajectory_marker_publisher/trajectory_marker_publisher.cpp
)

target_link_libraries(trajectory_marker_publisher ${catkin_LIBRARIES})
add_dependencies(trajectory_marker_publisher ${catkin_EXPORTED_TARGETS})

install(
  TARGETS
    trajectory_loader
    trajectory_replanner
    trajectory_saver
    trajectory_marker_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

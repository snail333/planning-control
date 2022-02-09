cmake_minimum_required(VERSION 2.8.3)
project(libtrajectory_follower)

find_package(catkin REQUIRED COMPONENTS
  common_msgs
  geometry_msgs
  roscpp
  roslint
  rosunit
  std_msgs
)

find_package(Eigen3 QUIET)

if (NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else ()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES libtrajectory_follower
  CATKIN_DEPENDS
    common_msgs
    geometry_msgs
    std_msgs
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(libtrajectory_follower
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(libtrajectory_follower src/libtrajectory_follower.cpp
  src/pure_pursuit.cpp)
add_dependencies(libtrajectory_follower ${catkin_EXPORTED_TARGETS})
target_link_libraries(libtrajectory_follower ${catkin_LIBRARIES})

## Install executables and/or libraries
install(TARGETS libtrajectory_follower
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

file(GLOB_RECURSE ROSLINT_FILES
  LIST_DIRECTORIES false
  *.cpp *.h *.hpp
)

list(APPEND ROSLINT_CPP_OPTS "--extensions=cc,h,hpp,cpp,cu,cuh" "--filter=-build/c++11,-runtime/references")
roslint_cpp(${ROSLINT_FILES})

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test-libtrajectory_follower
    test/test_libtrajectory_follower.test
    test/src/test_libtrajectory_follower.cpp
    src/libtrajectory_follower.cpp
  )
  add_dependencies(test-libtrajectory_follower ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test-libtrajectory_follower
    ${catkin_LIBRARIES}
  )
  add_rostest_gtest(test-pure_pursuit
    test/test_pure_pursuit.test
    test/src/test_pure_pursuit.cpp
    src/libtrajectory_follower.cpp
    src/pure_pursuit.cpp
  )  
  add_dependencies(test-pure_pursuit ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test-pure_pursuit
    ${catkin_LIBRARIES}
  )
  roslint_add_test()
endif ()

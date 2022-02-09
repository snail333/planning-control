cmake_minimum_required(VERSION 2.8.3)
project(llh2utm)
find_path(
  GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})

find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    roslint
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES llh2utm
  CATKIN_DEPENDS
)

add_definitions(${GeographicLib_DEFINITIONS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

add_library(llh2utm
  src/llh2utm.cpp
)

target_link_libraries(llh2utm
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)

file(GLOB_RECURSE ROSLINT_FILES
  LIST_DIRECTORIES false
  *.cpp *.h *.hpp
)

list(APPEND ROSLINT_CPP_OPTS
  "--extensions=cc,h,hpp,cpp,cu,cuh"
  "--filter=-build/c++14"
)

roslint_cpp(${ROSLINT_FILES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(TARGETS llh2utm
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

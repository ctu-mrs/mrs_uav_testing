cmake_minimum_required(VERSION 2.8.3)
project(mrs_uav_testing)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  mrs_msgs
  mrs_lib
  nav_msgs
  std_msgs
  std_srvs
  mavros_msgs
  )

catkin_package(
  CATKIN_DEPENDS nav_msgs mrs_lib std_msgs std_srvs mavros_msgs
  LIBRARIES ControlTest RandomFlier TrajectoryRandomFlier
  #  DEPENDS system_lib
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

## Declare a C++ executable
add_library(ControlTest src/control_test.cpp)

add_dependencies(ControlTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

## Specify libraries to link a library or executable target against
target_link_libraries(ControlTest
  ${catkin_LIBRARIES}
  )

## Declare a C++ executable
add_library(RandomFlier src/random_flier.cpp)

add_dependencies(RandomFlier
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

## Specify libraries to link a library or executable target against
target_link_libraries(RandomFlier
  ${catkin_LIBRARIES}
  )

## Declare a C++ executable
add_library(TrajectoryRandomFlier src/trajectory_random_flier.cpp)

add_dependencies(TrajectoryRandomFlier
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

## Specify libraries to link a library or executable target against
target_link_libraries(TrajectoryRandomFlier
  ${catkin_LIBRARIES}
  )

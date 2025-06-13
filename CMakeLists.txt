cmake_minimum_required(VERSION 3.12)
project(mrs_uav_testing)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall)
add_definitions(-Wextra)
add_definitions(-Wpedantic)

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

set(DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_components
  mrs_lib
  mrs_msgs
  tf2_ros
  backward_ros
  )

set(LIBRARIES
  MrsUavTesting_TestGeneric
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${mrs_lib_INCLUDE_DIRS}
  )

# Test Generic

add_library(MrsUavTesting_TestGeneric
  src/test_generic.cpp
  )

ament_target_dependencies(MrsUavTesting_TestGeneric
  rclcpp
  mrs_lib
  )

## --------------------------------------------------------------
## |                           Testing                          |
## --------------------------------------------------------------

if(ENABLE_TESTS)

  add_subdirectory(test)

endif()

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
)

install(TARGETS
  ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include
  DESTINATION .
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()

catkin_add_executable_with_gtest(test_takeoff
  test.cpp
  )

target_link_libraries(test_takeoff
  ${catkin_LIBRARIES}
  )

add_dependencies(test_takeoff
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

add_rostest(takeoff.test)

catkin_add_executable_with_gtest(test_midair
  test.cpp
  )

target_link_libraries(test_midair
  MrsUavTesting_TestGeneric
  ${catkin_LIBRARIES}
  )

add_dependencies(test_midair
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

add_rostest(midair_activation.test)

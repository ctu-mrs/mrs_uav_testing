catkin_add_executable_with_gtest(test_goto_relative
  test.cpp
  )

target_link_libraries(test_goto_relative
  MrsUavTesting_TestGeneric
  ${catkin_LIBRARIES}
  )

add_dependencies(test_goto_relative
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

add_rostest(goto_relative.test)

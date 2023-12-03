catkin_add_executable_with_gtest(test_goto
  test.cpp
  )

target_link_libraries(test_goto
  ${catkin_LIBRARIES}
  )

add_dependencies(test_goto
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

add_rostest(goto.test)

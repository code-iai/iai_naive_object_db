cmake_minimum_required(VERSION 2.8.3)
project(iai_naive_object_db)

find_package(catkin REQUIRED roscpp visualization_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES iai_naive_object_db
  CATKIN_DEPENDS roscpp visualization_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(TEST_SRCS
  test/main.cpp
  test/${PROJECT_NAME}/iai_naive_object_db.cpp
)

catkin_add_gtest(${PROJECT_NAME}-test ${TEST_SRCS}
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test_data)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
endif()

cmake_minimum_required(VERSION 2.8.3)
project(iai_naive_object_db)

find_package(catkin REQUIRED roscpp visualization_msgs geometry_msgs message_generation)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
    FILES
    Object.msg
    Visual.msg
)

add_service_files(
    FILES
    ObjectArray.srv
)

generate_messages(
  DEPENDENCIES
  visualization_msgs geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES iai_naive_object_db
  CATKIN_DEPENDS roscpp visualization_msgs geometry_msgs message_runtime
#  DEPENDS system_lib
)

#C++14
add_definitions("-std=c++1y")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y")

add_executable(iai_naive_object_db_main src/${PROJECT_NAME}/iai_naive_object_db_main.cpp)
target_link_libraries(iai_naive_object_db_main ${catkin_LIBRARIES})

add_executable(iai_naive_object_db_client_main src/${PROJECT_NAME}/iai_naive_object_db_client_main.cpp)
target_link_libraries(iai_naive_object_db_client_main ${catkin_LIBRARIES})

set(TEST_SRCS
  test/main.cpp
  test/${PROJECT_NAME}/iai_naive_object_db.cpp
)

catkin_add_gtest(${PROJECT_NAME}-test ${TEST_SRCS}
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test_data)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
endif()

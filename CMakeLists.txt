cmake_minimum_required(VERSION 3.0.2)
project(beginner_tutorials)

## Compile as C++14
set(CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf2_ros
)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(
    ${PROJECT_NAME}_test
    test/${PROJECT_NAME}_test.launch
    test/${PROJECT_NAME}_test.cpp
  )
  target_link_libraries(
    ${PROJECT_NAME}_test
    ${catkin_LIBRARIES}
  )
  add_dependencies(
    ${PROJECT_NAME}_test
    ${catkin_EXPORTED_TARGETS}
  )
endif()


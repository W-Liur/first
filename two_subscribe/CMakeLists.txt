cmake_minimum_required(VERSION 2.8.3)
project(two_subscribe)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
geometry_msgs
message_filters
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(two_subscribe src/two_subscribe.cpp)
target_link_libraries(two_subscribe ${catkin_LIBRARIES})
add_executable(two_send src/two_send.cpp)
target_link_libraries(two_send ${catkin_LIBRARIES})
add_executable(one_send src/one_send.cpp)
target_link_libraries(one_send ${catkin_LIBRARIES})

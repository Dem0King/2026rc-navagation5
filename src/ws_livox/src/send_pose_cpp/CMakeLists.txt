cmake_minimum_required(VERSION 3.0.2)
project(send_pose_cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  nav_msgs
  tf2
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(send_pose_node src/send_pose.cpp)
target_link_libraries(send_pose_node ${catkin_LIBRARIES})

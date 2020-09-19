cmake_minimum_required(VERSION 2.8.3)
project(trajectory_tracking)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES trajectory_tracking
 CATKIN_DEPENDS gazebo_ros geometry_msgs nav_msgs roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_executable(backstepping src/backstepping.cpp)
target_link_libraries(backstepping ${catkin_LIBRARIES})
add_dependencies(backstepping trajectory_tracking_generate_messages_cpp)

add_executable(feedback src/feedback.cpp)
target_link_libraries(feedback ${catkin_LIBRARIES})
add_dependencies(feedback trajectory_tracking_generate_messages_cpp)

add_executable(pid src/pid.cpp)
target_link_libraries(pid ${catkin_LIBRARIES})
add_dependencies(pid trajectory_tracking_generate_messages_cpp)

add_executable(mpc src/mpc.cpp)
target_link_libraries(mpc ${catkin_LIBRARIES})
add_dependencies(mpc trajectory_tracking_generate_messages_cpp)

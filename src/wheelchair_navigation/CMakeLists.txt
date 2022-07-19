cmake_minimum_required(VERSION 3.0.2)
project(wheelchair_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
	std_msgs
	nav_msgs
)

generate_messages(
	DEPENDENCIES
	std_msgs
	nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES abc_control
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# RRT PLANNER EXECUTABLE
add_executable(rrt_planner src/main.cpp)
target_link_libraries(rrt_planner ${catkin_LIBRARIES})
add_dependencies(rrt_planner wheelchair_navigation_gencpp)
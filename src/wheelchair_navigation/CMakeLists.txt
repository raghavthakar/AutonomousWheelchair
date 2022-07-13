cmake_minimum_required(VERSION 3.0.2)
project(wheelchair_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  nav_msgs
  std_msgs
  sensor_msgs
  actionlib_msgs
  geometry_msgs
  tf
)

## Generate actions in the 'action' folder
add_action_files(
FILES
MoveDistance.action
# Action2.action
)

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
	nav_msgs
	sensor_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs std_msgs geometry_msgs message_runtime 
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(rrt_planner src/main.cpp)
target_link_libraries(rrt_planner ${catkin_LIBRARIES})

add_executable(move_distance_server src/move_distance_obstacle_avoidance_server.cpp)
target_link_libraries(move_distance_server ${catkin_LIBRARIES})
add_dependencies(move_distance_server wheelchair_navigation_gencpp)

# INDIVIDUAL AGENT EXECUTABLE
add_executable(agent src/agent.cpp)
target_link_libraries(agent ${catkin_LIBRARIES})
add_dependencies(agent wheelchair_navigation_gencpp)
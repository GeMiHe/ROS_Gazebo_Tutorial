cmake_minimum_required(VERSION 2.8.3)
project(commander)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)

add_message_files(
	DIRECTORY msg
	FILES
		FireSource.msg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
)

catkin_package(
   CATKIN_DEPENDS
     roscpp
     rospy
     std_msgs
     geometry_msgs
     sensor_msgs
     nav_msgs
     message_runtime
   DEPENDS
     EIGEN3
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${src}
  ${EIGEN3_INCLUDE_DIR}
)


add_executable(example_node_kalman_filter src/example_node_kalman_filter.cpp)
target_link_libraries(example_node_kalman_filter ${catkin_LIBRARIES})
add_dependencies(example_node_kalman_filter ${catkin_EXPORTED_TARGETS})

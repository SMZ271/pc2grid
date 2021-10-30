cmake_minimum_required(VERSION 3.0.2)
project(pc2grid)

set(PCL_INCLUDE_DIRS /usr/include/pcl-1.8)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
  message_generation
  nav_msgs
)

find_package(PCL 1.8 REQUIRED COMPONENTS common io)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_message_files(
    FILES
    Grid.msg
)
generate_messages(
    DEPENDENCIES
    std_msgs
	)



catkin_package(
  CATKIN_DEPENDS message_runtime

	)

add_executable(pc_to_grid src/pc2grid.cpp)
target_link_libraries(pc_to_grid ${catkin_LIBRARIES} ${PCL_LIBRARIES})

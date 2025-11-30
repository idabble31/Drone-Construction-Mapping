cmake_minimum_required(VERSION 3.0.2)
project(drone_middleware)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
  tf2_sensor_msgs
  message_filters
  actionlib_msgs
  actionlib
)

find_package(PCL REQUIRED COMPONENTS common io filters)

add_action_files(
  DIRECTORY action
  FILES ScanMap.action
)

generate_messages(
  DEPENDENCIES std_msgs actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions tf2_ros tf2_sensor_msgs message_filters actionlib_msgs actionlib std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pcl_merge_node
  src/pcl_merge.cpp
)

target_link_libraries(pcl_merge_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_executable(merge_two_lidars src/merge_two_lidars.cpp)
target_link_libraries(merge_two_lidars ${catkin_LIBRARIES} ${PCL_LIBRARIES})

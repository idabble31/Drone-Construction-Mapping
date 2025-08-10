cmake_minimum_required(VERSION 3.0.2)
project(drone_middleware)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pcl_merge
  src/pcl_merge_node.cpp
  src/pcl_merge.cpp
)

target_link_libraries(pcl_merge
  ${catkin_LIBRARIES}
)

cmake_minimum_required(VERSION 3.0.2)
project(drone_middleware)

# 1. Force C++14 or C++17 (Required for modern PCL and ROS Noetic)
add_compile_options(-std=c++14)

# 2. Find Packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
  tf2_sensor_msgs
  message_filters
  actionlib_msgs
  actionlib
)

# Fix: Separate Boost and PCL. 
# Added 'system' and 'filesystem' to Boost for the directory creation logic.
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(PCL REQUIRED COMPONENTS common io filters)

# 3. Action Generation
add_action_files(
  DIRECTORY action
  FILES ScanMap.action
)

generate_messages(
  DEPENDENCIES std_msgs actionlib_msgs
)

# 4. Catkin Package Config
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions tf2_ros tf2_sensor_msgs message_filters actionlib_msgs actionlib std_msgs
)

# 5. Include Directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# CRITICAL: PCL Definitions (Prevents Segfaults with Eigen alignment)
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# --- TARGETS ---

# Node: PCL Merge
add_executable(pcl_merge_node src/pcl_merge.cpp)
target_link_libraries(pcl_merge_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# Safety: Wait for Actions/Msgs to generate before compiling
add_dependencies(pcl_merge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Node: Merge Two Lidars
add_executable(merge_two_lidars src/merge_two_lidars.cpp)
target_link_libraries(merge_two_lidars ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(merge_two_lidars ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Node: Map Saver
# Note: I changed source to 'src/map_saver_node.cpp' to match the code provided earlier.
# If your file is named 'map_saver.cpp', change it back below.
add_executable(map_saver_node src/map_saver.cpp)
target_link_libraries(map_saver_node 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES} # Links the filesystem library
)
add_dependencies(map_saver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
cmake_minimum_required(VERSION 3.0.2)
project(drone_middleware)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
  tf2_sensor_msgs
  message_filters
  image_transport
  cv_bridge
  std_msgs
  camera_info_manager
)

find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(OpenCV REQUIRED)
find_package(ZLIB REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    pcl_ros
    pcl_conversions
    tf2_ros
    tf2_sensor_msgs
    message_filters
    image_transport
    cv_bridge
    std_msgs
    camera_info_manager
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# ---------- Existing executables ----------
add_executable(pcl_merge_node src/pcl_merge.cpp)
target_link_libraries(pcl_merge_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(merge_two_lidars src/merge_two_lidars.cpp)
target_link_libraries(merge_two_lidars
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(pcd_saver_node src/pcd_saver.cpp)
target_link_libraries(pcd_saver_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(sensor_sync_node src/synchronize_sensor.cpp)
target_link_libraries(sensor_sync_node ${catkin_LIBRARIES})
add_dependencies(sensor_sync_node ${catkin_EXPORTED_TARGETS})

add_executable(sync_with_imu_interp src/sync_sensor_interp.cpp)
target_link_libraries(sync_with_imu_interp ${catkin_LIBRARIES})
add_dependencies(sync_with_imu_interp ${catkin_EXPORTED_TARGETS})

# ---------- Fisheye dewarp node (ROS1 Noetic) ----------
# Vendor cnpy (NPY reader): place cnpy.h/cnpy.cpp in third_party/cnpy
#   mkdir -p third_party/cnpy
#   curl -L https://raw.githubusercontent.com/rogersce/cnpy/master/cnpy.h -o third_party/cnpy/cnpy.h
#   curl -L https://raw.githubusercontent.com/rogersce/cnpy/master/cnpy.cpp -o third_party/cnpy/cnpy.cpp
add_library(cnpy STATIC thirdparty/cnpy/cnpy.cpp)
target_include_directories(cnpy PUBLIC thirdparty/cnpy)
target_link_libraries(cnpy PUBLIC ZLIB::ZLIB)

add_executable(dewarp_fisheye_node src/camera_dewarp.cpp)
target_link_libraries(dewarp_fisheye_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  cnpy
)

# ---------- Install targets ----------
install(TARGETS
  pcl_merge_node
  merge_two_lidars
  dewarp_fisheye_node
  pcd_saver_node
  sensor_sync_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

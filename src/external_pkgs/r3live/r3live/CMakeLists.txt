cmake_minimum_required(VERSION 2.8.3)
project(r3live)

# ---- Optional features -------------------------------------------------------
# Build-time toggle for Livox support (OFF = no livox_ros_driver dependency)
option(USE_LIVOX "Enable Livox ROS driver integration" OFF)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++14 -O3 -lboost_system -msse2 -msse3 -pthread -Wenum-compare") # -Wall
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -pthread -Wenum-compare")

# Only add SSE on x86/x86_64
if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|i.86")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -msse2 -msse3")
endif()

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Add OpenCV if available
# set(OpenCV_DIR "/home/zivlin/opt/opencv/build")
FIND_PACKAGE(Boost REQUIRED COMPONENTS filesystem iostreams program_options system serialization)
if(Boost_FOUND)
  INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
  LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
endif()

# Core catkin components (shared by both builds)
set(CORE_CATKIN_DEPS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
)
# Add Livox dependency only when enabled
if(USE_LIVOX)
  list(APPEND CORE_CATKIN_DEPS livox_ros_driver)
endif()

find_package(catkin REQUIRED COMPONENTS ${CORE_CATKIN_DEPS})

# find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

### Find OpenMP #######
FIND_PACKAGE(OpenMP)
if(OPENMP_FOUND)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  #cmake only check for separate OpenMP library on AppleClang 7+
  #https://github.com/Kitware/CMake/blob/42212f7539040139ecec092547b7d58ef12a4d72/Modules/FindOpenMP.cmake#L252
  if (CMAKE_CXX_COMPILER_ID MATCHES "AppleClang" AND (NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS "7.0"))
    SET(OpenMP_LIBS ${OpenMP_libomp_LIBRARY})
    LIST(APPEND OpenMVS_EXTRA_LIBS ${OpenMP_LIBS})
  endif()
else()
  message("-- Can't find OpenMP. Continuing without it.")
endif()

# find_package(OpenMVS)
if(OpenMVS_FOUND)
  include_directories(${OpenMVS_INCLUDE_DIRS})
  add_definitions(${OpenMVS_DEFINITIONS})
endif()

# message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

include_directories(${catkin_INCLUDE_DIRS})

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ./src
  ./src/loam/include
  ./src/tools/
  ./src/rgb_map 
  ./src/meshing
)

# Add Livox includes only when enabled (prevents hard dependency)
if(USE_LIVOX)
  include_directories(${livox_ros_driver_INCLUDE_DIRS})
endif()

catkin_package()

message(STATUS "===== ${PROJECT_NAME}: OpenCV library status: =====")
message(STATUS "===== OpenCV ersion: ${OpenCV_VERSION}  =====")
message(STATUS "===== OpenCV libraries: ${OpenCV_LIBS} =====")
message(STATUS "===== OpenCV include path: ${OpenCV_INCLUDE_DIRS} =====")
message(STATUS "===== Livox support (USE_LIVOX): ${USE_LIVOX} =====")

add_executable(r3live_LiDAR_front_end src/loam/LiDAR_front_end.cpp)
target_link_libraries(r3live_LiDAR_front_end ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# If Livox is enabled, expose the macro to relevant targets
if(USE_LIVOX)
  target_compile_definitions(r3live_LiDAR_front_end PRIVATE USE_LIVOX)
endif()

if(Ceres_FOUND)
  message(STATUS "===== Find ceres, Version ${Ceres_VERSION} =====")  
  include_directories(${CERES_INCLUDE_DIRS})
  add_executable(r3live_cam_cali src/r3live_cam_cali.cpp)
  target_link_libraries(r3live_cam_cali ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})

  add_executable(r3live_cam_cali_create_cali_board src/r3live_cam_cali_create_cali_board.cpp)
  target_link_libraries(r3live_cam_cali_create_cali_board ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
endif()
add_executable(test_timer src/tools/test_timer.cpp)

add_executable(r3live_mapping src/r3live.cpp 
                src/r3live_lio.cpp
                src/loam/include/kd_tree/ikd_Tree.cpp
                src/loam/include/FOV_Checker/FOV_Checker.cpp 
                src/loam/IMU_Processing.cpp
                src/rgb_map/offline_map_recorder.cpp
                # From VIO
                src/r3live_vio.cpp
                src/optical_flow/lkpyramid.cpp
                src/rgb_map/rgbmap_tracker.cpp
                src/rgb_map/image_frame.cpp
                src/rgb_map/pointcloud_rgbd.cpp
              )
target_link_libraries(r3live_mapping 
                          ${catkin_LIBRARIES}
                          ${Boost_LIBRARIES}
                          ${Boost_FILESYSTEM_LIBRARY}
                          ${Boost_SERIALIZATION_LIBRARY} # serialization
                          ${OpenCV_LIBRARIES}
                          # ${OpenMVS_LIBRARIES} 
			  ${PCL_LIBRARIES})

# Livox macro for main mapping target
if(USE_LIVOX)
  target_compile_definitions(r3live_mapping PRIVATE USE_LIVOX)
endif()

FIND_PACKAGE(CGAL REQUIRED)
if(CGAL_FOUND)
  include_directories(${CGAL_INCLUDE_DIRS})
  add_definitions(${CGAL_DEFINITIONS})
  link_directories(${CGAL_LIBRARY_DIRS})
  ADD_DEFINITIONS(-D_USE_BOOST -D_USE_EIGEN -D_USE_OPENMP)
  SET(_USE_BOOST TRUE)
  SET(_USE_OPENMP TRUE)
  SET(_USE_EIGEN TRUE)
  add_executable(r3live_meshing src/r3live_reconstruct_mesh.cpp                              
                                src/rgb_map/image_frame.cpp
                                src/rgb_map/pointcloud_rgbd.cpp  
                                # Common
                                src/meshing/MVS/Common/Common.cpp
                                src/meshing/MVS/Common/Log.cpp
                                src/meshing/MVS/Common/Timer.cpp
                                src/meshing/MVS/Common/Types.cpp
                                src/meshing/MVS/Common/Util.cpp
                                # MVS                       
                                src/meshing/MVS/Mesh.cpp
                                src/meshing/MVS/PointCloud.cpp
                                src/meshing/MVS/Camera.cpp
                                src/meshing/MVS/Platform.cpp
                                src/meshing/MVS/PLY.cpp
                                src/meshing/MVS/OBJ.cpp
                                src/meshing/MVS/IBFS.cpp
                                )
  target_link_libraries(r3live_meshing 
                                  ${catkin_LIBRARIES}
                                  ${Boost_LIBRARIES}
                                  ${Boost_FILESYSTEM_LIBRARY}
                                  ${Boost_SERIALIZATION_LIBRARY} # serialization
                                  ${CGAL_LIBS}
                                  ${OpenCV_LIBRARIES}
                                  ${JPEG_LIBRARIES} 
                                  gmp
                                  ${PCL_LIBRARIES})
endif()

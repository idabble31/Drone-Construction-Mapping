 cmake_minimum_required(VERSION 3.5)
 project(drone_description)

 find_package(catkin REQUIRED COMPONENTS
   rospy
   urdf
   xacro
   roscpp
   std_msgs
 )

 catkin_package(
   #  INCLUDE_DIRS include
   #  LIBRARIES drone_description
   #  CATKIN_DEPENDS rospy urdf xacro
   #  DEPENDS system_lib
 )

 include_directories(
   ${catkin_INCLUDE_DIRS}
 )

## Teleop C++ node
add_executable(teleop_drone_cpp src/teleop_drone.cpp)
target_link_libraries(teleop_drone_cpp ${catkin_LIBRARIES})
install(TARGETS teleop_drone_cpp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Scripts ##
#############

# Make sure your teleop script is executable and lives at scripts/drone_teleop.py
catkin_install_python(PROGRAMS
  scripts/teleop_drone.py
  scripts/motor_speed_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

 #############
 ## Install ##
 #############

 # if you have launch files or other resources, install them too:
 install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   FILES_MATCHING PATTERN "*.launch"
 )

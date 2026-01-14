cmake_minimum_required(VERSION 3.10)
project(Sophus)

set(CMAKE_VERBOSE_MAKEFILE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
  # add_definitions("-Wall -Werror -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
  ADD_DEFINITIONS("-Wall -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas -Wno-class-memaccess")
endif()

################################################################################
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_ROOT}/Modules")

set(Sophus_INCLUDE_DIR ${PROJECT_SOURCE_DIR})

################################################################################
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
set(Sophus_INCLUDE_DIR ${Sophus_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

set(SOURCE_DIR "sophus")
set(CLASSES so2 se2 se3 so3 scso3 sim3)

set(SOURCES)
foreach(class ${CLASSES})
  list(APPEND SOURCES ${SOURCE_DIR}/${class}.cpp ${SOURCE_DIR}/${class}.h)
endforeach()

add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${EIGEN3_LIBRARIES})

# Tests
add_executable(test_so2 sophus/test_so2.cpp)
add_executable(test_se2 sophus/test_se2.cpp)
add_executable(test_so3 sophus/test_so3.cpp)
add_executable(test_scso3 sophus/test_scso3.cpp)
add_executable(test_se3 sophus/test_se3.cpp)
add_executable(test_sim3 sophus/test_sim3.cpp)

target_link_libraries(test_so2 ${PROJECT_NAME})
target_link_libraries(test_se2 ${PROJECT_NAME})
target_link_libraries(test_so3 ${PROJECT_NAME})
target_link_libraries(test_scso3 ${PROJECT_NAME})
target_link_libraries(test_se3 ${PROJECT_NAME})
target_link_libraries(test_sim3 ${PROJECT_NAME})

enable_testing()
add_test(test_so2 test_so2)
add_test(test_se2 test_se2)
add_test(test_so3 test_so3)
add_test(test_scso3 test_scso3)
add_test(test_se3 test_se3)
add_test(test_sim3 test_sim3)

##############################################################################
# FIXED SECTION: Replacing the LOCATION property
# Instead of getting the path now, we let CMake handle the target name.
set(Sophus_LIBRARIES ${PROJECT_NAME})
set(Sophus_LIBRARY_DIR ${PROJECT_BINARY_DIR})

################################################################################
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/SophusConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/SophusConfig.cmake @ONLY IMMEDIATE)
export(PACKAGE Sophus)

install(DIRECTORY sophus DESTINATION include FILES_MATCHING PATTERN "*.h")
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

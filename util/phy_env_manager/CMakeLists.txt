cmake_minimum_required(VERSION 3.0.2)
project(phy_env_manager)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11  -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  Eigen3
  common
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  vehicle_model
  vehicle_msgs
  visualization_msgs
)

catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  inc
  ${common_INCLUDE_DIRS}
  ${vehicle_model_INCLUDE_DIRS}
  ${vehicle_msgs_INCLUDE_DIRS}
  ${semantics_msgs_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(phy_env_managing_node
    src/phy_env_manager/arena_loader.cc
    src/phy_env_manager/phy_manager.cc
    src/phy_env_manager/visualizer.cc
    src/phy_env_manager/ros_adapter.cc
    src/phy_env_managing_node.cc
)
target_link_libraries(phy_env_managing_node
   ${catkin_LIBRARIES}
)

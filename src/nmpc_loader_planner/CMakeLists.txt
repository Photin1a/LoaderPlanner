cmake_minimum_required(VERSION 3.0.2)
project(nmpc_loader_planner)
set(CMAKE_EXPORT_COMPILE_COMMANDS True)
# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)


find_package(casadi REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  decomp_util
  decomp_ros_utils
  decomp_ros_msgs
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

catkin_package(
 INCLUDE_DIRS include 
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf  decomp_util decomp_ros_utils decomp_ros_msgs
)

add_library(${PROJECT_NAME}
  src/nmpc_loader.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES} casadi)

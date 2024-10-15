cmake_minimum_required(VERSION 3.0.2)
project(nmpc_loader_planner)
set(CMAKE_EXPORT_COMPILE_COMMANDS True)

set(ENABLE_DEBUG ON)  #开启debug

# solver: ipopt or lbfgs. It is recommended to use ipopt. 
# In actual testing, the stability and quality of ipopt are better than those of lbfgs.
set(USE_IPOPT true)

if(ENABLE_DEBUG)
  set(CMAKE_BUILD_TYPE Debug)
else()
  set(CMAKE_BUILD_TYPE Release)
endif()

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

if(USE_IPOPT)
  add_definitions(-DUSE_IPOPT)
  add_library(${PROJECT_NAME}
    src/nmpc_loader_ipopt.cpp
  )
else() 
  add_library(${PROJECT_NAME}
  src/nmpc_loader_lbfgs.cpp
  )
endif()
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES} casadi)

cmake_minimum_required(VERSION 3.0.2)
project(simple_move_base)
# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS True)

find_package(catkin REQUIRED
    COMPONENTS
        geometry_msgs
        nav_msgs
        roscpp
        rospy
        tf2_ros
        hybrid_a_star
        nmpc_loader_planner
        decomp_util
)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})

# dynamic reconfigure
# generate_dynamic_reconfigure_options(
#     cfg/MoveBase.cfg
# )

catkin_package(
    INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIRS}
    LIBRARIES simple_move_base
    CATKIN_DEPENDS
        geometry_msgs
        nav_msgs
        roscpp
        tf2_ros
        hybrid_a_star
        nmpc_loader_planner
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# move_base
add_library(simple_move_base
  src/simple_move_base.cpp
)
target_link_libraries(simple_move_base
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )
add_dependencies(simple_move_base ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(simple_move_base_node
  src/simple_move_base_node.cpp
)
add_dependencies(simple_move_base_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_move_base_node simple_move_base)
# set_target_properties(simple_move_base_node PROPERTIES OUTPUT_NAME simple_move_base)

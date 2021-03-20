cmake_minimum_required(VERSION 2.8.3)
project(active_planner)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

catkin_package()

cs_add_executable(planner_node
        src/planner_node.cpp
        )
target_link_libraries(planner_node ${catkin_LIBRARIES})

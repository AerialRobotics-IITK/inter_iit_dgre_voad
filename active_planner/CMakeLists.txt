cmake_minimum_required(VERSION 2.8.3)
project(active_planner)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

add_definitions(-std=c++17)

cs_add_library(local_planner
        src/visualizer.cpp
        src/path_finder.cpp
        src/frontier_evaluator.cpp
        src/local_planner.cpp
)
target_link_libraries(local_planner ${catkin_LIBRARIES} ${BOOST_LIBRARIES} ${Eigen3_LIBRARIES})

cs_add_executable(local_planner_node
        src/local_planner_node.cpp
)
target_link_libraries(local_planner_node local_planner)

cs_install()
cs_export()
cmake_minimum_required(VERSION 3.0.2)
project(hdmap)

add_compile_options(
    -std=c++17 # for new future of c++
    -fconcepts # for auto paremeters in lambda
)

find_package(catkin REQUIRED COMPONENTS)

set(HDMAP_LIB_NAME "hdmap_lib")

# catkin_package(
#    INCLUDE_DIRS
#      ./
#    LIBRARIES
#      ${HDMAP_LIB_NAME}
# )

include(
    hdmap.h
)

include_directories(
    /usr/include/eigen3
    clothoids_wrapper/include
    ${catkin_INCLUDE_DIRS}
)

add_subdirectory(clothoids_wrapper)

add_library(${HDMAP_LIB_NAME}
    src/map/boundary.cc 
    src/map/lane.cc 
    src/map/map.cc 
    src/map/road.cc 
    src/map/segment.cc
    src/map/trajectory.cc
    src/routing/astar.cc
    src/routing/dijkstra.cc
    src/topograph/pathnode.cc
    src/topograph/topograph.cc
)

target_link_libraries(${HDMAP_LIB_NAME}
    ${catkin_LIBRARIES}
    clothoids_main
    clothoids_quartic_roots_flocke
    clothoids_utils
)

install(TARGETS ${HDMAP_LIB_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
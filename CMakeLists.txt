cmake_minimum_required(VERSION 3.0.2)
project(hdmap)

add_compile_options(
    -std=c++17 # for new future of c++
    -fconcepts # for auto paremeters in lambda
)

find_package(catkin REQUIRED COMPONENTS)

set(HDMAP_LIB_NAME "hdmap_lib")

add_subdirectory(depends/clothoids_wrapper)

include_directories(
    /usr/include/eigen3
    depends/clothoids_wrapper/include
    ${catkin_INCLUDE_DIRS}
)

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

# install(TARGETS 
#             ${HDMAP_LIB_NAME}
#             ${CLOTHOIDS_MAIN_LIBRARY_NAME}
#             ${CLOTHOIDS_UTILS_LIBRARY_NAME}
#             ${CLOTHOIDS_QUARTIC_ROOTS_FLOCKE_LIBRARY_NAME}
#     ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(FILE hdmap.h
#     DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

# install(DIRECTORY depends/clothoids_wrapper/include/${PROJECT_NAME}/
#         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )
cmake_minimum_required(VERSION 3.0.2)
project(clothoids_wrapper)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

set(CLOTHOIDS_MAIN_LIBRARY_NAME "clothoids_main")
set(CLOTHOIDS_UTILS_LIBRARY_NAME "clothoids_utils")
set(CLOTHOIDS_QUARTIC_ROOTS_FLOCKE_LIBRARY_NAME "clothoids_quartic_roots_flocke")

include_directories(
  include
)

# Main library
set(MAIN_LIB_SOURCE_DIRECTORY "src/${PROJECT_NAME}/${CLOTHOIDS_MAIN_LIBRARY_NAME}")
add_library(${CLOTHOIDS_MAIN_LIBRARY_NAME}
  ${MAIN_LIB_SOURCE_DIRECTORY}/AABBtree.cc
  ${MAIN_LIB_SOURCE_DIRECTORY}/Biarc.cc
  ${MAIN_LIB_SOURCE_DIRECTORY}/BiarcList.cc
  ${MAIN_LIB_SOURCE_DIRECTORY}/Circle.cc
  ${MAIN_LIB_SOURCE_DIRECTORY}/Clothoid.cc
  ${MAIN_LIB_SOURCE_DIRECTORY}/ClothoidAsyPlot.cc
  ${MAIN_LIB_SOURCE_DIRECTORY}/ClothoidDistance.cc
  ${MAIN_LIB_SOURCE_DIRECTORY}/ClothoidG2.cc
  ${MAIN_LIB_SOURCE_DIRECTORY}/ClothoidList.cc
  ${MAIN_LIB_SOURCE_DIRECTORY}/Fresnel.cc
  ${MAIN_LIB_SOURCE_DIRECTORY}/G2lib_intersect.cc
  ${MAIN_LIB_SOURCE_DIRECTORY}/G2lib.cc
  ${MAIN_LIB_SOURCE_DIRECTORY}/Line.cc
  ${MAIN_LIB_SOURCE_DIRECTORY}/PolyLine.cc
  ${MAIN_LIB_SOURCE_DIRECTORY}/Triangle2D.cc
)

# Utils library
set(UTILS_LIB_SOURCE_DIRECTORY "src/${PROJECT_NAME}/${CLOTHOIDS_UTILS_LIBRARY_NAME}")
add_library(${CLOTHOIDS_UTILS_LIBRARY_NAME}
  ${UTILS_LIB_SOURCE_DIRECTORY}/Console.cc
  ${UTILS_LIB_SOURCE_DIRECTORY}/CPUinfo.cc
  ${UTILS_LIB_SOURCE_DIRECTORY}/fmt.cc
  ${UTILS_LIB_SOURCE_DIRECTORY}/Malloc.cc
  ${UTILS_LIB_SOURCE_DIRECTORY}/Numbers.cc
  ${UTILS_LIB_SOURCE_DIRECTORY}/rang.cc
  ${UTILS_LIB_SOURCE_DIRECTORY}/Table.cc
  ${UTILS_LIB_SOURCE_DIRECTORY}/TicToc.cc
  ${UTILS_LIB_SOURCE_DIRECTORY}/Trace.cc
  ${UTILS_LIB_SOURCE_DIRECTORY}/Utils.cc
)

# Quartic roots flocke library
set(QUARTIC_LIB_SOURCE_DIRECTORY "src/${PROJECT_NAME}/${CLOTHOIDS_QUARTIC_ROOTS_FLOCKE_LIBRARY_NAME}")
add_library(${CLOTHOIDS_QUARTIC_ROOTS_FLOCKE_LIBRARY_NAME}
  ${QUARTIC_LIB_SOURCE_DIRECTORY}/PolynomialRoots-1-Quadratic.cc
  ${QUARTIC_LIB_SOURCE_DIRECTORY}/PolynomialRoots-2-Cubic.cc
  ${QUARTIC_LIB_SOURCE_DIRECTORY}/PolynomialRoots-3-Quartic.cc
  ${QUARTIC_LIB_SOURCE_DIRECTORY}/PolynomialRoots-Jenkins-Traub.cc
  ${QUARTIC_LIB_SOURCE_DIRECTORY}/PolynomialRoots-Utils.cc
)

# We need to add dllib to the utils library so the client doesn't need to link against it
target_link_libraries(${CLOTHOIDS_UTILS_LIBRARY_NAME}
                        ${CMAKE_DL_LIBS}
)
cmake_minimum_required(VERSION 3.12)
project(CollisionAlgorithm VERSION 0.1 LANGUAGES CXX)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

file(GLOB_RECURSE HEADER_FILES
    "src/*.h"
    "src/*.inl"
)

file(GLOB_RECURSE SOURCE_FILES
    "src/*.cpp"
)

file(GLOB_RECURSE SCENES_FILES
    "scenes/*.scn"
    "*.xml"
)

file(GLOB_RECURSE IGNORED_FILES
    "ignored/*.h"
    "ignored/*.inl"
    "ignored/*.cpp"
)

file(GLOB_RECURSE DEPRECATED_FILES
    "deprecated/*.h"
    "deprecated/*.inl"
    "deprecated/*.cpp"
)

find_package(Sofa.Simulation.Core REQUIRED)
find_package(Sofa.Component.StateContainer REQUIRED)
find_package(Sofa.Component.Constraint.Lagrangian.Solver REQUIRED)
find_package(Sofa.GL REQUIRED)


include_directories("${CMAKE_CURRENT_SOURCE_DIR}/ignored")
#include_directories("${CMAKE_CURRENT_SOURCE_DIR}/deprecated")

set_source_files_properties(${IGNORED_FILES} PROPERTIES HEADER_FILE_ONLY TRUE)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES} ${SCENES_FILES} ${IGNORED_FILES} ${DEPRECATED_FILES} ${README_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DPLUGIN_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\"")
target_link_libraries(${PROJECT_NAME}
    Sofa.Simulation.Core
    Sofa.Component.StateContainer
    Sofa.Component.Constraint.Lagrangian.Solver
    Sofa.GL
)

## Install rules for the library and headers; CMake package configurations files
set(${PROJECT_NAME}_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${${PROJECT_NAME}_INCLUDE_DIRS}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
sofa_create_package_with_targets(PACKAGE_NAME ${PROJECT_NAME} PACKAGE_VERSION 0.1 TARGETS ${PROJECT_NAME} INCLUDE_INSTALL_DIR ${${PROJECT_NAME}_INCLUDE_DIRS})

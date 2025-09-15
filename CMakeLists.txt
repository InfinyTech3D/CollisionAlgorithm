cmake_minimum_required(VERSION 3.12)
project(CollisionAlgorithm VERSION 0.1 LANGUAGES CXX)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

file(GLOB_RECURSE HEADER_FILES "src/*.h" "src/*.inl")
file(GLOB_RECURSE SOURCE_FILES "src/*.cpp")
file(GLOB_RECURSE SCENES_FILES "scenes/*.scn" "*.xml")
file(GLOB_RECURSE IGNORED_FILES "ignored/*.h" "ignored/*.inl" "ignored/*.cpp")
file(GLOB_RECURSE DEPRECATED_FILES "deprecated/*.h" "deprecated/*.inl" "deprecated/*.cpp")

find_package(Sofa.Simulation.Core REQUIRED)
find_package(Sofa.Component.StateContainer REQUIRED)
find_package(Sofa.Component.Constraint.Lagrangian.Solver REQUIRED)
find_package(Sofa.GL REQUIRED)

#include_directories("${CMAKE_CURRENT_SOURCE_DIR}/ignored")
#include_directories("${CMAKE_CURRENT_SOURCE_DIR}/deprecated")

set_source_files_properties(${IGNORED_FILES} PROPERTIES HEADER_FILE_ONLY TRUE)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES} ${README_FILES})

target_link_libraries(${PROJECT_NAME}
    Sofa.Simulation.Core
    Sofa.Component.StateContainer
    Sofa.Component.Constraint.Lagrangian.Solver
    Sofa.GL
)

sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME} 
    PACKAGE_VERSION ${PROJECT_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    EXAMPLE_INSTALL_DIR "scenes"
    RELOCATABLE "plugins"
    )

cmake_minimum_required(VERSION 3.12)
project(CollisionAlgorithm VERSION 0.1 LANGUAGES CXX)

find_package(Sofa.Simulation.Core REQUIRED)
find_package(Sofa.Component.StateContainer REQUIRED)
find_package(Sofa.Component.Constraint.Lagrangian.Solver REQUIRED)
find_package(Sofa.GL REQUIRED)

set(COLLISIONALGORITHM_SRC "src/${PROJECT_NAME}")

set(HEADER_FILES 
    ${COLLISIONALGORITHM_SRC}/config.h.in
    ${COLLISIONALGORITHM_SRC}/initCollisionAlgorithm.h

    ${COLLISIONALGORITHM_SRC}/BaseAABBBroadPhase.h
    ${COLLISIONALGORITHM_SRC}/BaseAlgorithm.h
    ${COLLISIONALGORITHM_SRC}/BaseElement.h
    ${COLLISIONALGORITHM_SRC}/BaseGeometry.h
    ${COLLISIONALGORITHM_SRC}/BaseOperation.h
    ${COLLISIONALGORITHM_SRC}/BaseProximity.h
    ${COLLISIONALGORITHM_SRC}/CollisionPipeline.h
    ${COLLISIONALGORITHM_SRC}/DataDetectionOutput.h
    ${COLLISIONALGORITHM_SRC}/ElementIterator.h
    ${COLLISIONALGORITHM_SRC}/InternalData.h

    ${COLLISIONALGORITHM_SRC}/algorithm/Find2DClosestProximityAlgorithm.h
    ${COLLISIONALGORITHM_SRC}/algorithm/FindClosestProximityAlgorithm.h
    ${COLLISIONALGORITHM_SRC}/algorithm/InsertionAlgorithm.h
    
    ${COLLISIONALGORITHM_SRC}/broadphase/AABBBroadPhase.h
    ${COLLISIONALGORITHM_SRC}/broadphase/FullAABBBroadPhase.h

    ${COLLISIONALGORITHM_SRC}/elements/EdgeElement.h
    ${COLLISIONALGORITHM_SRC}/elements/PointElement.h
    ${COLLISIONALGORITHM_SRC}/elements/TetrahedronElement.h
    ${COLLISIONALGORITHM_SRC}/elements/TriangleElement.h

    ${COLLISIONALGORITHM_SRC}/filters/DistanceFilter.h

    ${COLLISIONALGORITHM_SRC}/geometry/EdgeGeometry.h
    ${COLLISIONALGORITHM_SRC}/geometry/PointGeometry.h
    ${COLLISIONALGORITHM_SRC}/geometry/SubsetGeometry.h
    ${COLLISIONALGORITHM_SRC}/geometry/TetrahedronGeometry.h
    ${COLLISIONALGORITHM_SRC}/geometry/TriangleGeometry.h

    ${COLLISIONALGORITHM_SRC}/operations/CreateCenterProximity.h
    ${COLLISIONALGORITHM_SRC}/operations/FindClosestProximity.h
    ${COLLISIONALGORITHM_SRC}/operations/Project.h

    ${COLLISIONALGORITHM_SRC}/proximity/EdgeProximity.h
    ${COLLISIONALGORITHM_SRC}/proximity/FixedProximity.h
    ${COLLISIONALGORITHM_SRC}/proximity/MechanicalProximity.h
    ${COLLISIONALGORITHM_SRC}/proximity/MultiProximity.h
    ${COLLISIONALGORITHM_SRC}/proximity/PointProximity.h
    ${COLLISIONALGORITHM_SRC}/proximity/TetrahedronProximity.h
    ${COLLISIONALGORITHM_SRC}/proximity/TriangleProximity.h

    ${COLLISIONALGORITHM_SRC}/toolbox/EdgeToolBox.h
    ${COLLISIONALGORITHM_SRC}/toolbox/PointToolBox.h
    ${COLLISIONALGORITHM_SRC}/toolbox/TetrahedronToolBox.h
    ${COLLISIONALGORITHM_SRC}/toolbox/TriangleToolBox.h
)

set(SOURCE_FILES
    ${COLLISIONALGORITHM_SRC}/initCollisionAlgorithm.cpp

    ${COLLISIONALGORITHM_SRC}/CollisionPipeline.cpp
    
    ${COLLISIONALGORITHM_SRC}/algorithm/Find2DClosestProximityAlgorithm.cpp
    ${COLLISIONALGORITHM_SRC}/algorithm/FindClosestProximityAlgorithm.cpp
    ${COLLISIONALGORITHM_SRC}/algorithm/InsertionAlgorithm.cpp

    ${COLLISIONALGORITHM_SRC}/broadphase/AABBBroadPhase.cpp
    ${COLLISIONALGORITHM_SRC}/broadphase/FullAABBBroadPhase.cpp

    ${COLLISIONALGORITHM_SRC}/elements/EdgeElement.cpp
    ${COLLISIONALGORITHM_SRC}/elements/PointElement.cpp
    ${COLLISIONALGORITHM_SRC}/elements/TetrahedronElement.cpp
    ${COLLISIONALGORITHM_SRC}/elements/TriangleElement.cpp

    ${COLLISIONALGORITHM_SRC}/filters/DistanceFilter.cpp

    ${COLLISIONALGORITHM_SRC}/geometry/EdgeGeometry.cpp
    ${COLLISIONALGORITHM_SRC}/geometry/PointGeometry.cpp
    ${COLLISIONALGORITHM_SRC}/geometry/SubsetGeometry.cpp
    ${COLLISIONALGORITHM_SRC}/geometry/TetrahedronGeometry.cpp
    ${COLLISIONALGORITHM_SRC}/geometry/TriangleGeometry.cpp

    ${COLLISIONALGORITHM_SRC}/operations/CreateCenterProximity.cpp
    ${COLLISIONALGORITHM_SRC}/operations/FindClosestProximity.cpp
    ${COLLISIONALGORITHM_SRC}/operations/Project.cpp

    ${COLLISIONALGORITHM_SRC}/toolbox/EdgeToolBox.cpp
    ${COLLISIONALGORITHM_SRC}/toolbox/PointToolBox.cpp
    ${COLLISIONALGORITHM_SRC}/toolbox/TetrahedronToolBox.cpp
    ${COLLISIONALGORITHM_SRC}/toolbox/TriangleToolBox.cpp
)

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

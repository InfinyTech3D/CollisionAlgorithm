#pragma once

#include <CollisionAlgorithm/config.h>
#include <CollisionAlgorithm/BaseProximity.h>
#include <CollisionAlgorithm/elements/EdgeElement.h>
#include <CollisionAlgorithm/operations/Project.h>
#include <CollisionAlgorithm/operations/CreateCenterProximity.h>

namespace sofa::collisionalgorithm::toolbox {

class SOFA_COLLISIONALGORITHM_API EdgeToolBox {
public:

    static Operations::CreateCenterProximity::Result createCenterProximity(const EdgeElement::SPtr & edge);

    static Operations::Project::Result project(const type::Vec3 & P, const EdgeElement::SPtr & edge);

    static void projectOnEdge(const type::Vec3d & projP, const type::Vec3d & e1, const type::Vec3d & e2, double & fact_u, double & fact_v);

    static void normalize(const type::Vec3d & P0, const type::Vec3d & P1,double & f0,double & f1);

};



}


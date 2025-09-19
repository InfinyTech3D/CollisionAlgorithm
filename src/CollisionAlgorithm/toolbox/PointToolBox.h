#pragma once

#include <CollisionAlgorithm/BaseProximity.h>
#include <CollisionAlgorithm/elements/PointElement.h>
#include <CollisionAlgorithm/operations/Project.h>
#include <CollisionAlgorithm/operations/CreateCenterProximity.h>

namespace sofa::collisionalgorithm::toolbox {

class PointToolBox {
public:

    static Operations::CreateCenterProximity::Result createCenterProximity(const PointElement::SPtr & point);

    static Operations::Project::Result project(const type::Vec3 & P, const PointElement::SPtr & point);

};



}


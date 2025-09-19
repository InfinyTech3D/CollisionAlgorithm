#pragma once

#include <CollisionAlgorithm/config.h>
#include <CollisionAlgorithm/BaseProximity.h>
#include <CollisionAlgorithm/elements/TriangleElement.h>
#include <CollisionAlgorithm/operations/Project.h>
#include <CollisionAlgorithm/operations/CreateCenterProximity.h>
#include <CollisionAlgorithm/operations/ContainsPoint.h>

namespace sofa::collisionalgorithm::toolbox {

class SOFA_COLLISIONALGORITHM_API TriangleToolBox {
public:

    static Operations::CreateCenterProximity::Result createCenterProximity(const TriangleElement::SPtr & tri);

    static Operations::ContainsPointInElement::Result containsPoint(const type::Vec3 & P, const TriangleElement::SPtr & tri);

    static Operations::Project::Result project(const type::Vec3 & P, const TriangleElement::SPtr & tri);

    static void computeTriangleBaryCoords(const type::Vec3d & proj_P, const TriangleElement::TriangleInfo & tinfo, double & fact_u, double & fact_v, double & fact_w);

    static void projectOnTriangle(const type::Vec3d projectP, const TriangleElement::TriangleInfo & tinfo, double & fact_u, double & fact_v, double & fact_w);

    static bool isInTriangle(const type::Vec3d & P, const TriangleElement::TriangleInfo & tinfo, double & fact_u, double & fact_v, double & fact_w);

    static void normalize(const type::Vec3d & P0, const type::Vec3d & P1, const type::Vec3d & P2,double & f0,double & f1, double & f2);

};

}

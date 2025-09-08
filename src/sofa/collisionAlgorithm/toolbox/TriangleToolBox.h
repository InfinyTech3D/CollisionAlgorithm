#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>
#include <sofa/collisionAlgorithm/operations/Project.h>
#include <sofa/collisionAlgorithm/operations/CreateCenterProximity.h>

namespace sofa::collisionAlgorithm::toolbox {

class TriangleToolBox {
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


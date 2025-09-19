#pragma once

#include <CollisionAlgorithm/BaseProximity.h>
#include <CollisionAlgorithm/elements/TetrahedronElement.h>
#include <CollisionAlgorithm/operations/Project.h>
#include <CollisionAlgorithm/operations/CreateCenterProximity.h>
#include <CollisionAlgorithm/operations/ContainsPoint.h>

namespace sofa::collisionalgorithm::toolbox {

class TetrahedronToolBox {
public:

    static Operations::CreateCenterProximity::Result createCenterProximity(const TetrahedronElement::SPtr & tetra);

    static Operations::ContainsPointInElement::Result containsPoint(const type::Vec3 & P, const TetrahedronElement::SPtr & tetra);

    static Operations::Project::Result project(const type::Vec3 & P, const TetrahedronElement::SPtr & tetra);

    static void projectOnTetra(const type::Vec3d projectP, const TetrahedronElement::TetraInfo & teinfo, double & fact_u, double & fact_v, double & fact_w,double & fact_x);

    static void computeTetraBaryCoords(const type::Vec3d & P, const TetrahedronElement::TetraInfo & tinfo, double & fact_u,double & fact_v, double & fact_w, double & fact_x);

    static bool isInTetra(const type::Vec3d & P, const TetrahedronElement::TetraInfo & tinfo, double & fact_u,double & fact_v, double & fact_w, double & fact_x);

    static void normalize(const type::Vec3d & P0, const type::Vec3d & P1, const type::Vec3d & P2, const type::Vec3d & P3, double & f0,double & f1, double & f2,double & f3);

};

}

#include <CollisionAlgorithm/toolbox/EdgeToolBox.h>
#include <CollisionAlgorithm/proximity/EdgeProximity.h>

namespace sofa::collisionalgorithm::toolbox {

Operations::CreateCenterProximity::Result EdgeToolBox::createCenterProximity(const EdgeElement::SPtr & edge) {
    return EdgeProximity::create(edge,0.5,0.5);
}

Operations::Project::Result EdgeToolBox::project(const type::Vec3 & P, const EdgeElement::SPtr & edge) {
    double fact_u,fact_v;
    const type::Vec3 P0 = EdgeProximity::create(edge,1,0)->getPosition();
    const type::Vec3 P1 = EdgeProximity::create(edge,0,1)->getPosition();

    projectOnEdge(P,P0,P1,fact_u,fact_v);

    BaseProximity::SPtr prox = EdgeProximity::create(edge,fact_u,fact_v);
    BaseProximity::SPtr prox_normalized = prox->copy();
    prox_normalized->normalize();

    double dist = (P-prox_normalized->getPosition()).norm();

    return Operations::Project::Result(dist,prox);
}

void EdgeToolBox::projectOnEdge(const type::Vec3d & projP, const type::Vec3d & e1, const type::Vec3d & e2, double & fact_u, double & fact_v) {
    type::Vec3d v = e2 - e1;
    fact_v = dot(projP - e1,v) / dot(v,v);
    fact_u = 1.0-fact_v;
}

void EdgeToolBox::normalize(const type::Vec3d & P0, const type::Vec3d & P1,
                      double & f0,double & f1) {

    type::Vec3 P = (P0*f0)+(P1*f1);

    toolbox::EdgeToolBox::projectOnEdge(P,
                                        P0, P1,
                                        f0, f1);

    if (f1<0.0) f1 = 0.0;
    else if (f1>1.0) f1 = 1.0;

    f0 = 1.0-f1;
}

}


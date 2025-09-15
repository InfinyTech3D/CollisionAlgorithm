#include <CollisionAlgorithm/toolbox/TriangleToolBox.h>
#include <CollisionAlgorithm/toolbox/EdgeToolBox.h>
#include <CollisionAlgorithm/proximity/TriangleProximity.h>

namespace sofa::collisionalgorithm::toolbox {

Operations::CreateCenterProximity::Result TriangleToolBox::createCenterProximity(const TriangleElement::SPtr & tri) {
    return TriangleProximity::create(tri, 1.0/3.0,1.0/3.0,1.0/3.0);
}

Operations::ContainsPointInElement::Result TriangleToolBox::containsPoint(const type::Vec3 & P, const TriangleElement::SPtr & tri) {
    TriangleProximity::SPtr prox = TriangleProximity::create(tri, 1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0);
    double f0(prox->f0()), f1(prox->f1()), f2(prox->f2());
    return isInTriangle(P,tri->getTriangleInfo(),f0,f1,f1);
}

//Barycentric coordinates are computed according to
//http://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
Operations::Project::Result TriangleToolBox::project(const type::Vec3 & P, const TriangleElement::SPtr & tri) {
    double fact_u,fact_v,fact_w;
    projectOnTriangle(P,tri->getTriangleInfo(),fact_u,fact_v,fact_w);

    TriangleProximity::SPtr prox = TriangleProximity::create(tri, fact_u,fact_v,fact_w);
    BaseProximity::SPtr prox_normalized = prox->copy();
    prox_normalized->normalize();

    double dist = (prox_normalized->getPosition() - P).norm();

    return Operations::Project::Result(dist,prox);
}

void TriangleToolBox::computeTriangleBaryCoords(const type::Vec3d & proj_P, const TriangleElement::TriangleInfo & tinfo, double & fact_u, double & fact_v, double & fact_w) {
    type::Vec3d v2 = proj_P - tinfo.P0;

    double d20 = dot(v2,tinfo.v0);
    double d21 = dot(v2,tinfo.v1);

    fact_v = (tinfo.d11 * d20 - tinfo.d01 * d21) * tinfo.invDenom;
    fact_w = (tinfo.d00 * d21 - tinfo.d01 * d20) * tinfo.invDenom;
    fact_u = 1.0 - fact_v  - fact_w;
}

void TriangleToolBox::projectOnTriangle(const type::Vec3d projectP, const TriangleElement::TriangleInfo & tinfo, double & fact_u, double & fact_v, double & fact_w) {
    type::Vec3d x1x2 = projectP - tinfo.P0;

    //corrdinate on the plane
    double c0 = dot(x1x2,tinfo.ax1);
    double c1 = dot(x1x2,tinfo.ax2);
    type::Vec3d proj_P = tinfo.P0 + tinfo.ax1 * c0 + tinfo.ax2 * c1;

    computeTriangleBaryCoords(proj_P, tinfo, fact_u,fact_v,fact_w);
}

bool TriangleToolBox::isInTriangle(const type::Vec3d & P, const TriangleElement::TriangleInfo & tinfo, double & fact_u, double & fact_v, double & fact_w) {
    computeTriangleBaryCoords(P, tinfo, fact_u, fact_v, fact_w);
    if ((fact_u<0 || fact_u>1) || (fact_v<0 || fact_v>1) || (fact_w<0 || fact_w>1)) return false;
    return true;
}

void TriangleToolBox::normalize(const type::Vec3d & P0, const type::Vec3d & P1, const type::Vec3d & P2, double & f0,double & f1, double & f2) {
    if (f0<0) {
        type::Vec3 P = (P0*f0)+(P1*f1)+(P2*f2);

        toolbox::EdgeToolBox::projectOnEdge(P,
                                            P1, P2,
                                            f1, f2);

        f0=0;
        toolbox::EdgeToolBox::normalize(P1,P2,f1,f2);
    } else if (f1<0) {
        type::Vec3 P = (P0*f0)+(P1*f1)+(P2*f2);

        toolbox::EdgeToolBox::projectOnEdge(P,
                                            P0, P2,
                                            f0, f2);
        f1=0;
        toolbox::EdgeToolBox::normalize(P0,P2,f0,f2);
    } else if (f2<0) {
        type::Vec3 P = (P0*f0)+(P1*f1)+(P2*f2);

        toolbox::EdgeToolBox::projectOnEdge(P,
                                            P0, P1,
                                            f0, f1);
        f2=0;
        toolbox::EdgeToolBox::normalize(P0,P1,f0,f1);
    }
}

}



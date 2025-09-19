#include <CollisionAlgorithm/toolbox/TetrahedronToolBox.h>
#include <CollisionAlgorithm/elements/TetrahedronElement.h>
#include <CollisionAlgorithm/proximity/TetrahedronProximity.h>
#include <CollisionAlgorithm/operations/Project.h>
#include <CollisionAlgorithm/operations/CreateCenterProximity.h>

namespace sofa::collisionalgorithm::toolbox {

Operations::CreateCenterProximity::Result TetrahedronToolBox::createCenterProximity(const TetrahedronElement::SPtr & tetra) {
    return TetrahedronProximity::create(tetra, 1.0/4.0,1.0/4.0,1.0/4.0,1.0/4.0);
}

Operations::ContainsPointInElement::Result TetrahedronToolBox::containsPoint(const type::Vec3 & P, const TetrahedronElement::SPtr & tetra) {
    TetrahedronProximity::SPtr prox = TetrahedronProximity::create(tetra, 1.0/4.0,1.0/4.0,1.0/4.0,1.0/4.0);
    double f0(prox->f0()), f1(prox->f1()), f2(prox->f2()), f3(prox->f3());
    return isInTetra(P,tetra->getTetrahedronInfo(),f0,f1,f2,f3);
}

Operations::Project::Result TetrahedronToolBox::project(const type::Vec3 & P, const TetrahedronElement::SPtr & tetra) {
    double fact[4];
    projectOnTetra(P, tetra->getTetrahedronInfo(),fact[0],fact[1],fact[2],fact[3]);

    BaseProximity::SPtr prox = TetrahedronProximity::create(tetra, fact[0],fact[1],fact[2],fact[3]);
	BaseProximity::SPtr prox_normalized = prox->copy();
    prox_normalized->normalize();

    double dist = (prox_normalized->getPosition() - P).norm();

    return Operations::Project::Result(dist,prox);
}

void TetrahedronToolBox::projectOnTetra(const type::Vec3d projectP, const TetrahedronElement::TetraInfo & teinfo, double & fact_u, double & fact_v, double & fact_w,double & fact_x) {
    computeTetraBaryCoords(projectP, teinfo, fact_u,fact_v,fact_w,fact_x);
}

void TetrahedronToolBox::computeTetraBaryCoords(const type::Vec3d & P, const TetrahedronElement::TetraInfo & tinfo, double & fact_u,double & fact_v, double & fact_w, double & fact_x) {
    type::Vec3d e = P - tinfo.P0;

    double Va = 1.0/6.0 * dot(e,tinfo.ax2Cax3);
    double Vb = 1.0/6.0 * dot(tinfo.ax1,e.cross(tinfo.ax3));
    double Vc = 1.0/6.0 * dot(tinfo.ax1,tinfo.ax2.cross(e));

    fact_v = Va/tinfo.V0;
    fact_w = Vb/tinfo.V0;
    fact_x = Vc/tinfo.V0;
    fact_u = 1.0 - (fact_v + fact_w + fact_x);
}


bool TetrahedronToolBox::isInTetra(const type::Vec3d & P, const TetrahedronElement::TetraInfo & tinfo, double & fact_u,double & fact_v, double & fact_w, double & fact_x) {
    computeTetraBaryCoords(P, tinfo, fact_u, fact_v, fact_w, fact_x);
    if ((fact_u<0 || fact_u>1) || (fact_v<0 || fact_v>1) || (fact_w<0 || fact_w>1) || (fact_x<0 || fact_x>1)) return false;
    return true;
}


void TetrahedronToolBox::normalize(const type::Vec3d & P0, const type::Vec3d & P1, const type::Vec3d & P2, const type::Vec3d & P3, double & f0,double & f1, double & f2,double & f3) {
    if(f0<0) {
        type::Vec3 P = (P0*f0)+(P1*f1)+(P2*f2)+(P3*f3);

        TriangleElement::TriangleInfo tinfo;
        tinfo.update(P1,P2,P3);
        toolbox::TriangleToolBox::projectOnTriangle(P, tinfo, f1,f2,f3);

        f0 = 0;
        toolbox::TriangleToolBox::normalize(P1,P2,P3,
                                            f1,f2,f3);
    } else if(f1<0) {
        type::Vec3 P = (P0*f0)+(P1*f1)+(P2*f2)+(P3*f3);

        TriangleElement::TriangleInfo tinfo;
        tinfo.update(P0,P2,P3);

        toolbox::TriangleToolBox::projectOnTriangle(P, tinfo,f0,f2,f3);

        f1 = 0;
        toolbox::TriangleToolBox::normalize(P0,P2,P3,
                                            f0,f2,f3);
    } else if(f2<0) {
        type::Vec3 P = (P0*f0)+(P1*f1)+(P2*f2)+(P3*f3);

        TriangleElement::TriangleInfo tinfo;
        tinfo.update(P0,P1,P3);

        toolbox::TriangleToolBox::projectOnTriangle(P, tinfo,f0,f1,f3);

        f2 = 0;
        toolbox::TriangleToolBox::normalize(P0,P1,P3,
                                            f0,f1,f3);
    } else if(f3<0) {
        type::Vec3 P = (P0*f0)+(P1*f1)+(P2*f2)+(P3*f3);

        TriangleElement::TriangleInfo tinfo;
        tinfo.update(P0,P1,P2);

        toolbox::TriangleToolBox::projectOnTriangle(P, tinfo,f0,f1,f2);

        f3 = 0;
        toolbox::TriangleToolBox::normalize(P0,P1,P2,
                                            f0,f1,f2);
    }
}

}


#pragma once

#include <CollisionAlgorithm/BaseAlgorithm.h>
#include <CollisionAlgorithm/BaseGeometry.h>
#include <CollisionAlgorithm/BaseOperation.h>
#include <CollisionAlgorithm/operations/CreateCenterProximity.h>
#include <CollisionAlgorithm/operations/Project.h>
#include <CollisionAlgorithm/operations/FindClosestProximity.h>

namespace sofa::collisionalgorithm {

class FindClosestProximityAlgorithm : public BaseAlgorithm {
public:
    SOFA_CLASS(FindClosestProximityAlgorithm, BaseAlgorithm);

    core::objectmodel::SingleLink<FindClosestProximityAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_from;
    core::objectmodel::SingleLink<FindClosestProximityAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_dest;
    Data<bool> d_drawCollision ;
    Data<DetectionOutput<BaseProximity,BaseProximity> > d_output;
    Data<bool> d_projective ;
//    Data<sofa::type::vector<double> > d_outputDist;

    FindClosestProximityAlgorithm()
    : l_from(initLink("fromGeom", "link to from geometry"))
    , l_dest(initLink("destGeom", "link to dest geometry"))
    , d_drawCollision (initData(&d_drawCollision, true, "drawcollision", "draw collision"))
    , d_output(initData(&d_output,"output", "output of the collision detection"))
    , d_projective(initData(&d_projective, false,"projective", "projection of closest prox onto from element"))
//    , d_outputDist(initData(&d_outputDist,"outputDist", "Distance of the outpu pair of detections"))
    {}

    void draw(const core::visual::VisualParams* vparams) {
        if (! vparams->displayFlags().getShowCollisionModels() && ! d_drawCollision.getValue()) return;
        glDisable(GL_LIGHTING);
        glColor4f(0,1,0,1);

        glBegin(GL_LINES);
        DetectionOutput output = d_output.getValue() ;
        for (unsigned i=0;i<output.size();i++) {
            glVertex3dv(output[i].first->getPosition().data());
            glVertex3dv(output[i].second->getPosition().data());
        }
        glEnd();
    }

    void doDetection() {
        if (l_from == NULL) return;
        if (l_dest == NULL) return;

        auto & output = *d_output.beginEdit();
        output.clear();

        auto itfrom = l_from->begin();

        auto createProximityOp = Operations::CreateCenterProximity::Operation::get(itfrom->getTypeInfo());
        auto findClosestProxOp = Operations::FindClosestProximity::Operation::get(l_dest);
        auto projectOp = Operations::Project::Operation::get(l_dest);
        auto projectFromOp = Operations::Project::Operation::get(l_from);

        for (;itfrom!=l_from->end();itfrom++) {
            auto pfrom = createProximityOp(itfrom->element());
            if (pfrom == nullptr) continue;

            auto pdest = findClosestProxOp(pfrom, l_dest.get(), projectOp, getFilterFunc());
            if (pdest == nullptr) continue;
            pdest->normalize();


            if (d_projective.getValue()) {
                //auto pfromProj = projectFromOp(pdest->getPosition(),itfrom->element()).prox;
                auto projectOnShaft = Operations::Project::Operation::get(l_from);
                auto findClosestProxOnShaft = Operations::FindClosestProximity::Operation::get(l_from);
                auto pfromProj = findClosestProxOnShaft(pdest, l_from, projectOnShaft, getFilterFunc());
                if (pfromProj == nullptr) continue;
                pfromProj->normalize();

                output.add(pfromProj,pdest);
            } else {
                output.add(pfrom,pdest);
            }

        }

        d_output.endEdit();
    }

};

}


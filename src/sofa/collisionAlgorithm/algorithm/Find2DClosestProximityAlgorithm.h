#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/collisionAlgorithm/operations/CreateCenterProximity.h>
#include <sofa/collisionAlgorithm/operations/Project.h>
#include <sofa/collisionAlgorithm/operations/FindClosestProximity.h>

namespace sofa::collisionAlgorithm {

class Find2DClosestProximityAlgorithm : public BaseAlgorithm {
public:
    SOFA_CLASS(Find2DClosestProximityAlgorithm, BaseAlgorithm);

    core::objectmodel::SingleLink<Find2DClosestProximityAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_from;
    core::objectmodel::SingleLink<Find2DClosestProximityAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_dest;

    Data<bool> d_drawCollision ;

//    typedef type::Mat3x4d Mat3x4d;
    Data<sofa::type::Mat3x4d> d_projectionMatrix;


    Data<DetectionOutput<BaseProximity,BaseProximity> > d_output;
//    Data<sofa::type::vector<double> > d_outputDist;

    Find2DClosestProximityAlgorithm()
    : l_from(initLink("from", "link to from geometry"))
    , l_dest(initLink("dest", "link to dest geometry"))
    , d_drawCollision (initData(&d_drawCollision, true, "drawcollision", "draw collision"))
    , d_projectionMatrix(initData(&d_projectionMatrix,
                                  sofa::type::Mat3x4d(sofa::type::Vec<4,float>(5972.63, 557.773, 288.39, 1.11926e+06),
                                                      sofa::type::Vec<4,float>(14.2385, 3000.63, -5203.53, -2.08222e+06 ),
                                                      sofa::type::Vec<4,float>(-0.0211254, 0.905754, 0.423277, 3041.26)),
                                                      "projectionMatrix", "projection matrix"))

//                                  Mat3x4d(type::Vec<4,float>(1.0,0.0,0.0,0.0),
//                                          type::Vec<4,float>(0.0,1.0,0.0,0.0),
//                                          type::Vec<4,float>(0.0,0.0,1.0,0.0)),


    , d_output(initData(&d_output,"output", "output of the collision detection"))
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


    //Project 3d to 2d
    sofa::type::Vec2 project(const type::Vec3 & p) const {

        const sofa::type::Mat3x4d & P = d_projectionMatrix.getValue();

        double rx = P[0][0] * p[0] + P[0][1] * p[1] + P[0][2] * p[2] + P[0][3];
        double ry = P[1][0] * p[0] + P[1][1] * p[1] + P[1][2] * p[2] + P[1][3];
        double rz = P[2][0] * p[0] + P[2][1] * p[1] + P[2][2] * p[2] + P[2][3];

        return sofa::type::Vec2 (rx,ry) * 1.0/rz;
    }


    //Find the Closest Proximity in projected 2D
    BaseProximity::SPtr findClosestProximity2D(BaseProximity::SPtr prox, ElementIterator::SPtr itdest) {

        double min_dist = std::numeric_limits<double>::max();
        BaseProximity::SPtr res = NULL;

        type::Vec3 P = prox->getPosition();
        Operations::Project::Operation::FUNC projectOp = Operations::Project::Operation::get(itdest->getTypeInfo());

        for (; !itdest->end(); itdest++) {
            auto edest = itdest->element();
            if (edest == nullptr) continue;

            BaseProximity::SPtr pdest = projectOp(prox->getPosition(),edest).prox;
            if (pdest == NULL) continue;
            pdest->normalize();

            sofa::type::Vec2 a = project(P);
            sofa::type::Vec2 b = project(pdest->getPosition());

            double d = (a-b).norm();
            if (d < min_dist)
            {
                res = pdest;
                min_dist = d;
            }
        }

        return res;
    }


    void doDetection() {
        if (l_from == NULL) return;
        if (l_dest == NULL) return;

        auto & output = *d_output.beginEdit();
        output.clear();

        auto itfrom = l_from->pointBegin();

        auto createProximityOp = Operations::CreateCenterProximity::Operation::get(l_from->begin()->getTypeInfo());
//        auto findClosestProxOp = Operations::FindClosestProximity::Operation::get(l_dest);
//        auto projectOp = Operations::Project::Operation::get(l_dest);

        for (;itfrom!=l_from->end();itfrom++) {
            auto itdest = l_dest->pointBegin();

            auto pfrom = createProximityOp(itfrom->element());
            if (pfrom == nullptr) continue;

            auto pdest = findClosestProximity2D(pfrom,itdest);
            if (pdest == nullptr) continue;

            pdest->normalize();

            output.add(pfrom,pdest);
        }

        d_output.endEdit();
    }

};

}


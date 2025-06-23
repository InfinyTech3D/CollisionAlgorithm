#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/collisionAlgorithm/operations/CreateCenterProximity.h>
#include <sofa/collisionAlgorithm/operations/Project.h>
#include <sofa/collisionAlgorithm/operations/FindClosestProximity.h>
#include <sofa/component/statecontainer/MechanicalObject.h>
#include <sofa/component/constraint/lagrangian/solver/ConstraintSolverImpl.h>

namespace sofa::collisionAlgorithm {

class PunctureAlgorithm : public BaseAlgorithm {
public:
    SOFA_CLASS(PunctureAlgorithm, BaseAlgorithm);

    core::objectmodel::SingleLink<PunctureAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_from;
    core::objectmodel::SingleLink<PunctureAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_dest;
    Data<bool> d_drawCollision ;
    Data<DetectionOutput<BaseProximity,BaseProximity> > d_output;
    Data<bool> d_projective ;
    Data<SReal> d_punctureThreshold ;
//    Data<sofa::type::vector<double> > d_outputDist;
    sofa::component::constraint::lagrangian::solver::ConstraintSolverImpl* m_constraintSolver;

    PunctureAlgorithm()
    : l_from(initLink("fromGeom", "link to from geometry"))
    , l_dest(initLink("destGeom", "link to dest geometry"))
    , d_drawCollision (initData(&d_drawCollision, true, "drawcollision", "draw collision"))
    , d_output(initData(&d_output,"output", "output of the collision detection"))
    , d_projective(initData(&d_projective, false,"projective", "projection of closest prox onto from element"))
    , d_punctureThreshold(initData(&d_punctureThreshold, std::numeric_limits<double>::max(), "punctureThreshold", "Threshold for puncture detection"))
//    , d_outputDist(initData(&d_outputDist,"outputDist", "Distance of the outpu pair of detections"))
    , m_constraintSolver(nullptr)
    {}

    void init() override {
        BaseAlgorithm::init();
        m_constraintSolver
            = this->getContext()->get<sofa::component::constraint::lagrangian::solver::ConstraintSolverImpl>();
    }

    void draw(const core::visual::VisualParams* vparams) {
        if (! vparams->displayFlags().getShowCollisionModels() && ! d_drawCollision.getValue()) return;
        vparams->drawTool()->disableLighting();

        DetectionOutput output = d_output.getValue() ;
        for (unsigned i=0;i<output.size();i++) {
            vparams->drawTool()->drawLine(
                output[i].first->getPosition(), 
                output[i].second->getPosition(), 
                sofa::type::RGBAColor(0, 1, 0, 1)
            );
        }

    }

    void doDetection() {
        if (l_from == NULL) return;
        if (l_dest == NULL) return;

        auto& output = *d_output.beginEdit();
        output.clear();

        const sofa::component::statecontainer::MechanicalObject<defaulttype::RigidTypes>* mstate
            = l_from->getContext()->get<sofa::component::statecontainer::MechanicalObject<defaulttype::RigidTypes>>();
        if (m_constraintSolver)
        {
            defaulttype::RigidTypes::Vec3 lambda = 
                m_constraintSolver->getLambda()[mstate].read()->getValue()[0].getVCenter();
            if (lambda.norm() > d_punctureThreshold.getValue()) return;// Exit and leave output empty
        }

        auto itfrom = l_from->begin();

        auto createProximityOp = Operations::CreateCenterProximity::Operation::get(itfrom->getTypeInfo());
        auto findClosestProxOp = Operations::FindClosestProximity::Operation::get(l_dest);
        auto projectOp = Operations::Project::Operation::get(l_dest);
        auto projectFromOp = Operations::Project::Operation::get(l_from);

        for (; itfrom != l_from->end(); itfrom++) {
            auto pfrom = createProximityOp(itfrom->element());
            if (pfrom == nullptr) continue;

            auto pdest = findClosestProxOp(pfrom, l_dest.get(), projectOp, getFilterFunc());
            if (pdest == nullptr) continue;
            pdest->normalize();

            if (d_projective.getValue()) {
                auto pfromProj = projectFromOp(pdest->getPosition(), itfrom->element()).prox;
                if (pfromProj == nullptr) continue;
                pfromProj->normalize();

                output.add(pfromProj, pdest);
            }
            else {
                output.add(pfrom, pdest);
            }

        }

        d_output.endEdit();
    }

};

}


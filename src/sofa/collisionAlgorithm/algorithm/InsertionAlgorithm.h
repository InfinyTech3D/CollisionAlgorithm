#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/collisionAlgorithm/operations/CreateCenterProximity.h>
#include <sofa/collisionAlgorithm/operations/Project.h>
#include <sofa/collisionAlgorithm/operations/FindClosestProximity.h>
#include <sofa/component/statecontainer/MechanicalObject.h>
#include <sofa/component/constraint/lagrangian/solver/ConstraintSolverImpl.h>

#include <sofa/collisionAlgorithm/proximity/TetrahedronProximity.h>

namespace sofa::collisionAlgorithm {

class InsertionAlgorithm : public BaseAlgorithm {
public:
    SOFA_CLASS(InsertionAlgorithm, BaseAlgorithm);

    core::objectmodel::SingleLink<InsertionAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_from;
    core::objectmodel::SingleLink<InsertionAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_dest;
    core::objectmodel::SingleLink<InsertionAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_destVol;
    Data<bool> d_drawCollision ;
    Data<bool> d_drawPoints ;
    Data<SReal> d_sphereRadius ;
    Data<DetectionOutput<BaseProximity,BaseProximity> > d_output;
    Data<DetectionOutput<BaseProximity,BaseProximity> > d_outputList;
    Data<bool> d_projective ;
    Data<SReal> d_punctureThreshold ;
    Data<SReal> d_slideDistance ;
//    Data<sofa::type::vector<double> > d_outputDist;
    sofa::component::constraint::lagrangian::solver::ConstraintSolverImpl* m_constraintSolver;
    std::vector<BaseProximity::SPtr> m_proximities;
    std::vector<TetrahedronElement::SPtr> m_tetras;

    InsertionAlgorithm()
    : l_from(initLink("fromGeom", "link to from geometry"))
    , l_dest(initLink("destGeom", "link to dest geometry"))
    , l_destVol(initLink("destVol", "link to dest geometry (volume)"))
    , d_drawCollision (initData(&d_drawCollision, true, "drawcollision", "draw collision"))
    , d_drawPoints(initData(&d_drawPoints, true, "drawPoints", "draw detection outputs"))
    , d_sphereRadius(initData(&d_sphereRadius, 0.001, "sphereRadius", "radius for drawing detection outputs"))
    , d_output(initData(&d_output,"output", "output of the collision detection"))
    , d_outputList(initData(&d_outputList,"outputList", "output of the detection inside the volume"))
    , d_projective(initData(&d_projective, false,"projective", "projection of closest prox onto from element"))
    , d_punctureThreshold(initData(&d_punctureThreshold, std::numeric_limits<double>::max(), "punctureThreshold", "Threshold for puncture detection"))
    , d_slideDistance(initData(&d_slideDistance, std::numeric_limits<double>::min(), "slideDistance", "Distance along the insertion trajectory after which the proximities slide backwards along the needle shaft"))
//    , d_outputDist(initData(&d_outputDist,"outputDist", "Distance of the outpu pair of detections"))
    , m_constraintSolver(nullptr)
    , m_tetras()
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
        for (const auto& it : output) {
            vparams->drawTool()->drawLine(it.first->getPosition(), it.second->getPosition(), sofa::type::RGBAColor(0, 1, 0, 1));
        }

        DetectionOutput outputList = d_outputList.getValue() ;
        for (const auto& it : outputList) {
            vparams->drawTool()->drawSphere(it.first->getPosition(),  d_sphereRadius.getValue(), sofa::type::RGBAColor(1, 0, 0, 1));
            vparams->drawTool()->drawSphere(it.second->getPosition(), d_sphereRadius.getValue(), sofa::type::RGBAColor(0, 0, 1, 1));
            vparams->drawTool()->drawLine(it.first->getPosition(), it.second->getPosition(), sofa::type::RGBAColor(1, 1, 0, 1));
        }

        for(const auto& it : m_tetras) {
            vparams->drawTool()->drawTetrahedron(
                it->getP0()->getPosition(),
                it->getP1()->getPosition(),
                it->getP2()->getPosition(),
                it->getP3()->getPosition(),
                sofa::type::RGBAColor(1, 1, 0, 0.3)
            );
        }

    }

    void doDetection() {
        if (l_from == NULL) return;
        if (l_dest == NULL) return;

        auto& output = *d_output.beginEdit();
        auto& outputList = *d_outputList.beginEdit();

        const sofa::component::statecontainer::MechanicalObject<defaulttype::Vec3Types>* mstate
            = l_from->getContext()->get<sofa::component::statecontainer::MechanicalObject<defaulttype::Vec3Types>>();
        if (mstate->getSize() > 1)
        {
            msg_warning() << "Requested MechanicalObject, corresponding to the tip of the needle in the InsertionAlgorithm, has a size greater than 1. "
                        << "The algorithm is designed to work with a single point. Only the first element will be used.";
        }
        if (m_constraintSolver)
        {
            defaulttype::Vec3Types::VecCoord lambda =
                m_constraintSolver->getLambda()[mstate].read()->getValue();
            if (lambda[0].norm() > d_punctureThreshold.getValue())
            {
                for (const auto& dpair : output) {
                    outputList.add(dpair.first->copy(), dpair.second->copy());
                    //m_proximities.push_back(itOutputPair.second->copy());
                }
                output.clear();
                return;
            }
        }

        output.clear();
        //outputList.clear();

        auto itfrom = l_from->begin();

        auto createProximityOp = Operations::CreateCenterProximity::Operation::get(itfrom->getTypeInfo());
        auto findClosestProxOp = Operations::FindClosestProximity::Operation::get(l_dest);
        auto projectOp = Operations::Project::Operation::get(l_dest);
        auto projectVolOp = Operations::Project::Operation::get(l_destVol);
        auto projectFromOp = Operations::Project::Operation::get(l_from);

        auto findClosestProxOpVol = Operations::FindClosestProximity::Operation::get(l_destVol);

        for (; itfrom != l_from->end(); itfrom++) 
        {
            auto pfrom = createProximityOp(itfrom->element());
            if (pfrom == nullptr) continue;

            if (outputList.size() == 0)
            {
                auto pdest = findClosestProxOp(pfrom, l_dest.get(), projectOp, getFilterFunc());
                if (pdest != nullptr)
                {
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
            }
            else
            {
                const SReal dist = (pfrom->getPosition() - outputList.back().second->getPosition()).norm();
                if(dist > d_slideDistance.getValue()) 
                {
                    auto pdestVol = findClosestProxOpVol(pfrom, l_destVol.get(), projectVolOp, getFilterFunc());
                    if (pdestVol != nullptr) 
                    {
                        /*
                        TetrahedronProximity::SPtr tetraProx = std::dynamic_pointer_cast<TetrahedronProximity>(pdestVol);
                        double* baryCoords = tetraProx->getBaryCoord();
                        if (toolbox::TetrahedronToolBox::isInTetra(
                                pfrom->getPosition(),
                                tetraProx->element()->getTetrahedronInfo(),
                                baryCoords[0],
                                baryCoords[1],
                                baryCoords[2],
                                baryCoords[3]
                            )
                        ) m_tetras.push_back(tetraProx->element());
                        */

                        pdestVol->normalize();

                        if (d_projective.getValue()) {
                            auto pfromProj = projectFromOp(pdestVol->getPosition(), itfrom->element()).prox;
                            if (pfromProj == nullptr) continue;
                            pfromProj->normalize();

                            outputList.add(pfromProj, pdestVol);
                        }
                        else {
                            outputList.add(pfrom, pdestVol);
                        }
                    }
                }
            }
        }

        d_output.endEdit();
        d_outputList.endEdit();
    }

};

}


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
    Data<DetectionOutput<BaseProximity,BaseProximity> > d_output;
    Data<DetectionOutput<BaseProximity,BaseProximity> > d_outputInside;
    Data<bool> d_projective ;
    Data<SReal> d_punctureThreshold ;
//    Data<sofa::type::vector<double> > d_outputDist;
    sofa::component::constraint::lagrangian::solver::ConstraintSolverImpl* m_constraintSolver;
    std::vector<BaseProximity::SPtr> m_proximities;
    std::vector<TetrahedronElement::SPtr> m_tetras;

    InsertionAlgorithm()
    : l_from(initLink("fromGeom", "link to from geometry"))
    , l_dest(initLink("destGeom", "link to dest geometry"))
    , l_destVol(initLink("destVol", "link to dest geometry (volume)"))
    , d_drawCollision (initData(&d_drawCollision, true, "drawcollision", "draw collision"))
    , d_output(initData(&d_output,"output", "output of the collision detection"))
    , d_outputInside(initData(&d_outputInside,"outputInside", "output of the detection inside the volume"))
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

        DetectionOutput outputInside = d_outputInside.getValue() ;
        for (unsigned i=0;i<outputInside.size();i++) {
            vparams->drawTool()->drawLine(
                outputInside[i].first->getPosition(), 
                outputInside[i].second->getPosition(), 
                sofa::type::RGBAColor(1, 1, 0, 1)
            );
        }

        for(unsigned i = 0; i < m_tetras.size(); ++i) {
            vparams->drawTool()->drawTetrahedron(
                m_tetras[i]->getP0()->getPosition(),
                m_tetras[i]->getP1()->getPosition(),
                m_tetras[i]->getP2()->getPosition(),
                m_tetras[i]->getP3()->getPosition(),
                sofa::type::RGBAColor(1, 1, 0, 1)
            );
        }

    }

    void doDetection() {
        if (l_from == NULL) return;
        if (l_dest == NULL) return;

        auto& output = *d_output.beginEdit();
        auto& outputInside = *d_outputInside.beginEdit();

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
                for (const auto& itOutputPair : output) {
                    m_proximities.push_back(itOutputPair.second->copy());
                }
                output.clear();
                return;
            }
        }

        output.clear();
        outputInside.clear();

        auto itfrom = l_from->begin();

        auto createProximityOp = Operations::CreateCenterProximity::Operation::get(itfrom->getTypeInfo());
        auto findClosestProxOp = Operations::FindClosestProximity::Operation::get(l_dest);
        auto projectOp = Operations::Project::Operation::get(l_dest);
        auto projectVolOp = Operations::Project::Operation::get(l_destVol);
        auto projectFromOp = Operations::Project::Operation::get(l_from);

        for (; itfrom != l_from->end(); itfrom++) {
            auto pfrom = createProximityOp(itfrom->element());
            if (pfrom == nullptr) continue;

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

            auto findClosestProxOpVol = Operations::FindClosestProximity::Operation::get(l_destVol);
            auto pdestVol = findClosestProxOpVol(pfrom, l_destVol.get(), projectVolOp, getFilterFunc());
            if (pdestVol != nullptr) 
            {
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
                )
                {
                    m_tetras.push_back(tetraProx->element());
                }

                pdestVol->normalize();
    
                if (d_projective.getValue()) {
                    auto pfromProj = projectFromOp(pdestVol->getPosition(), itfrom->element()).prox;
                    if (pfromProj == nullptr) continue;
                    pfromProj->normalize();
    
                    outputInside.add(pfromProj, pdestVol);
                }
                else {
                    outputInside.add(pfrom, pdestVol);
                }
            }
        }

        d_output.endEdit();
        d_outputInside.endEdit();
    }

};

}


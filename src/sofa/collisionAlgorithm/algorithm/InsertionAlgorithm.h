#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/collisionAlgorithm/operations/CreateCenterProximity.h>
#include <sofa/collisionAlgorithm/operations/Project.h>
#include <sofa/collisionAlgorithm/operations/FindClosestProximity.h>
#include <sofa/component/statecontainer/MechanicalObject.h>
#include <sofa/component/constraint/lagrangian/solver/ConstraintSolverImpl.h>

#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>

namespace sofa::collisionAlgorithm {

class InsertionAlgorithm : public BaseAlgorithm {
public:
    SOFA_CLASS(InsertionAlgorithm, BaseAlgorithm);

    core::objectmodel::SingleLink<InsertionAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_from;
    core::objectmodel::SingleLink<InsertionAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_dest;
    core::objectmodel::SingleLink<InsertionAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_fromVol;
    core::objectmodel::SingleLink<InsertionAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_destVol;
    Data<bool> d_drawCollision ;
    Data<bool> d_drawPoints ;
    Data<SReal> d_sphereRadius ;
    Data<DetectionOutput<BaseProximity,BaseProximity> > d_output;
    Data<DetectionOutput<BaseProximity,BaseProximity> > d_outputList;
    Data<bool> d_projective ;
    Data<SReal> d_punctureThreshold ;
    Data<SReal> d_slideDistance ;
    sofa::component::constraint::lagrangian::solver::ConstraintSolverImpl* m_constraintSolver;
    std::vector<BaseProximity::SPtr> m_needlePts;
    std::vector<BaseProximity::SPtr> m_couplingPts;

    InsertionAlgorithm()
    : l_from(initLink("fromGeom", "link to from geometry"))
    , l_dest(initLink("destGeom", "link to dest geometry"))
    , l_fromVol(initLink("fromVol", "link to from geometry (volume)"))
    , l_destVol(initLink("destVol", "link to dest geometry (volume)"))
    , d_drawCollision (initData(&d_drawCollision, true, "drawcollision", "draw collision"))
    , d_drawPoints(initData(&d_drawPoints, true, "drawPoints", "draw detection outputs"))
    , d_sphereRadius(initData(&d_sphereRadius, 0.0005, "sphereRadius", "radius for drawing detection outputs"))
    , d_output(initData(&d_output,"output", "output of the collision detection"))
    , d_outputList(initData(&d_outputList,"outputList", "output of the detection inside the volume"))
    , d_projective(initData(&d_projective, false,"projective", "projection of closest prox onto from element"))
    , d_punctureThreshold(initData(&d_punctureThreshold, std::numeric_limits<double>::max(), "punctureThreshold", "Threshold for puncture detection"))
    , d_slideDistance(initData(&d_slideDistance, std::numeric_limits<double>::min(), "slideDistance", "Distance along the insertion trajectory after which the proximities slide backwards along the needle shaft"))
    , m_constraintSolver(nullptr)
    , m_needlePts()
    , m_couplingPts()
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
            vparams->drawTool()->drawSphere(it.first->getPosition(),  d_sphereRadius.getValue(), sofa::type::RGBAColor(1, 0, 1, 0.9));
            vparams->drawTool()->drawSphere(it.second->getPosition(), d_sphereRadius.getValue(), sofa::type::RGBAColor(0, 0, 1, 0.9));
            vparams->drawTool()->drawLine(it.first->getPosition(), it.second->getPosition(), sofa::type::RGBAColor(1, 1, 0, 1));
        }
    }

    void doDetection() {
        if (l_from == NULL) return;
        if (l_dest == NULL) return;
        if (l_fromVol == NULL) return;
        if (l_destVol == NULL) return;

        auto& output = *d_output.beginEdit();
        auto& outputList = *d_outputList.beginEdit();

        if (outputList.size() == 0) 
        {
            const sofa::core::behavior::MechanicalState<defaulttype::Vec3Types>* mstate
                = l_from->getContext()->get<sofa::core::behavior::MechanicalState<defaulttype::Vec3Types>>();
            if (m_constraintSolver)
            {
                defaulttype::Vec3Types::VecCoord lambda =
                    m_constraintSolver->getLambda()[mstate].read()->getValue();
                if (lambda[0].norm() > d_punctureThreshold.getValue())
                {
                    auto findClosestProxOp_needle = Operations::FindClosestProximity::Operation::get(l_fromVol);
                    auto projectOp_needle = Operations::Project::Operation::get(l_fromVol);
                    for (const auto& dpair : output)
                    {
                        // Reproject onto the needle to create an EdgeProximity - The EdgeHandler requires this
                        auto pfromVol = findClosestProxOp_needle(dpair.second, l_fromVol.get(), projectOp_needle, getFilterFunc());
                        m_needlePts.push_back(pfromVol);
                        m_couplingPts.push_back(dpair.second->copy());
                        outputList.add(pfromVol, dpair.second->copy());
                    }
                    output.clear();
                    return;
                }
            }

            output.clear();

            auto itfrom = l_from->begin();

            auto createProximityOp = Operations::CreateCenterProximity::Operation::get(itfrom->getTypeInfo());
            auto findClosestProxOp = Operations::FindClosestProximity::Operation::get(l_dest);
            auto projectOp = Operations::Project::Operation::get(l_dest);
            auto projectFromOp = Operations::Project::Operation::get(l_from);

            for (; itfrom != l_from->end(); itfrom++) 
            {
                auto pfrom = createProximityOp(itfrom->element());
                if (pfrom == nullptr) continue;
                auto pdest = findClosestProxOp(pfrom, l_dest.get(), projectOp, getFilterFunc());
                if (pdest != nullptr) {
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
        }
        else
        {
            outputList.clear();

            auto itfrom = l_from->begin();
            auto createProximityOp = Operations::CreateCenterProximity::Operation::get(itfrom->getTypeInfo());
            auto pfrom = createProximityOp(itfrom->element());

            auto itfromVol = l_fromVol->begin(l_fromVol->getSize() - 2);
            auto createProximityOpVol = Operations::CreateCenterProximity::Operation::get(itfromVol->getTypeInfo());
            auto pfromVol = createProximityOpVol(itfromVol->element());
            const EdgeProximity::SPtr edgeProx = dynamic_pointer_cast<EdgeProximity>(pfromVol);
            const type::Vec3 normal = (edgeProx->element()->getP1()->getPosition() - edgeProx->element()->getP0()->getPosition()).normalized();
            type::Vec3 ab = m_couplingPts.back()->getPosition() - pfrom->getPosition();
            const SReal dotProd = dot(ab, normal);
            if (dotProd > 0.0)
            {
                m_couplingPts.pop_back();
                m_needlePts.pop_back();
            }

            const SReal dist = ab.norm();
            if(dist > d_slideDistance.getValue()) 
            {
                auto findClosestProxOp_vol = Operations::FindClosestProximity::Operation::get(l_destVol);
                auto projectOp_vol = Operations::Project::Operation::get(l_destVol);
                auto projectFromOp_vol = Operations::Project::Operation::get(l_fromVol);
                auto pdestVol = findClosestProxOp_vol(pfrom, l_destVol.get(), projectOp_vol, getFilterFunc());
                if (pdestVol)
                {
                    pdestVol->normalize();
                    m_couplingPts.push_back(pdestVol);
                    m_needlePts.push_back(m_needlePts.back());
                }
            }

            auto findClosestProxOp_needle = Operations::FindClosestProximity::Operation::get(l_fromVol);
            auto projectOp_needle = Operations::Project::Operation::get(l_fromVol);

            for(int i = 0 ; i < m_couplingPts.size(); i++)
            {
                auto pfromVol = findClosestProxOp_needle(m_couplingPts[i], l_fromVol.get(), projectOp_needle, getFilterFunc());
                m_needlePts[i] = pfromVol;
            }

            for(int i = 0 ; i < m_couplingPts.size(); i++)
                outputList.add(m_needlePts[i], m_couplingPts[i]);
        }

        d_output.endEdit();
        d_outputList.endEdit();
    }

};

}


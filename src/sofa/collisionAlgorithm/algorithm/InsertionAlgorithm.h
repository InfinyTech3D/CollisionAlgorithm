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

    typedef core::behavior::MechanicalState<defaulttype::Vec3Types> MechStateTipType;
    typedef core::objectmodel::SingleLink<InsertionAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> GeomLink;
    typedef DetectionOutput<BaseProximity,BaseProximity> AlgorithmOutput;
    typedef component::constraint::lagrangian::solver::ConstraintSolverImpl ConstraintSolver;

    GeomLink l_tipGeom, l_surfGeom, l_shaftGeom, l_volGeom;
    Data<AlgorithmOutput> d_collisionOutput, d_insertionOutput;
    Data<bool> d_projective ;
    Data<SReal> d_punctureThreshold, d_slideDistance;
    ConstraintSolver* m_constraintSolver;
    std::vector<BaseProximity::SPtr> m_needlePts, m_couplingPts;
    Data<bool> d_drawCollision, d_drawPoints ;
    Data<SReal> d_drawPointsScale ;

    InsertionAlgorithm()
    : l_tipGeom(initLink("tipGeom", "link to the geometry structure of the needle tip"))
    , l_surfGeom(initLink("surfGeom", "link to the geometry of the surface punctured by the needle"))
    , l_shaftGeom(initLink("shaftGeom", "link to the geometry structure of the needle shaft"))
    , l_volGeom(initLink("volGeom", "link to the geometry of volume wherein the needle is inserted"))
    , d_collisionOutput(initData(&d_collisionOutput,"collisionOutput", "detected proximities during puncture"))
    , d_insertionOutput(initData(&d_insertionOutput,"insertionOutput", "detected proximities during insertion"))
    , d_projective(initData(&d_projective, false,"projective", "projection of closest prox onto from element"))
    , d_punctureThreshold(initData(&d_punctureThreshold, std::numeric_limits<double>::max(), "punctureThreshold", "Threshold for puncture detection"))
    , d_slideDistance(initData(&d_slideDistance, std::numeric_limits<double>::min(), "slideDistance", "Distance along the insertion trajectory after which the proximities slide backwards along the needle shaft"))
    , m_constraintSolver(nullptr)
    , m_needlePts()
    , m_couplingPts()
    , d_drawCollision (initData(&d_drawCollision, false, "drawcollision", "draw collision"))
    , d_drawPoints(initData(&d_drawPoints, false, "drawPoints", "draw detection outputs"))
    , d_drawPointsScale(initData(&d_drawPointsScale, 0.0005, "drawPointsScale", "scale the drawing of detection output points"))
    {}

    void init() override {
        BaseAlgorithm::init();
        m_constraintSolver = this->getContext()->get<ConstraintSolver>();
    }

    void draw(const core::visual::VisualParams* vparams) {
        if (! vparams->displayFlags().getShowCollisionModels() && ! d_drawCollision.getValue()) return;
        vparams->drawTool()->disableLighting();

        DetectionOutput collisionOutput = d_collisionOutput.getValue() ;
        for (const auto& it : collisionOutput) {
            vparams->drawTool()->drawLine(it.first->getPosition(), it.second->getPosition(), type::RGBAColor(0, 1, 0, 1));
        }

        DetectionOutput insertionOutput = d_insertionOutput.getValue() ;
        for (const auto& it : insertionOutput) {
            vparams->drawTool()->drawSphere(it.first->getPosition(),  d_drawPointsScale.getValue(), type::RGBAColor(1, 0, 1, 0.9));
            vparams->drawTool()->drawSphere(it.second->getPosition(), d_drawPointsScale.getValue(), type::RGBAColor(0, 0, 1, 0.9));
            vparams->drawTool()->drawLine(it.first->getPosition(), it.second->getPosition(), type::RGBAColor(1, 1, 0, 1));
        }
    }

    void doDetection() {
        if (l_tipGeom == NULL) return;
        if (l_surfGeom == NULL) return;
        if (l_shaftGeom == NULL) return;
        if (l_volGeom == NULL) return;

        auto& collisionOutput = *d_collisionOutput.beginEdit();
        auto& insertionOutput = *d_insertionOutput.beginEdit();

        if (insertionOutput.size() == 0) 
        {
                const MechStateTipType* mstate = l_tipGeom->getContext()->get<MechStateTipType>();
            if (m_constraintSolver)
            {
                const auto lambda = m_constraintSolver->getLambda()[mstate].read()->getValue();
                if (lambda[0].norm() > d_punctureThreshold.getValue())
                {
                    auto findClosestProxOp_needle = Operations::FindClosestProximity::Operation::get(l_shaftGeom);
                    auto projectOp_needle = Operations::Project::Operation::get(l_shaftGeom);
                    for (const auto& dpair : collisionOutput)
                    {
                        // Reproject onto the needle to create an EdgeProximity - The EdgeHandler requires this
                        auto pfromVol = findClosestProxOp_needle(dpair.second, l_shaftGeom.get(), projectOp_needle, getFilterFunc());
                        m_needlePts.push_back(pfromVol);
                        m_couplingPts.push_back(dpair.second->copy());
                        insertionOutput.add(pfromVol, dpair.second->copy());
                    }
                    collisionOutput.clear();
                    return;
                }
            }

            collisionOutput.clear();

            auto itfrom = l_tipGeom->begin();

            auto createProximityOp = Operations::CreateCenterProximity::Operation::get(itfrom->getTypeInfo());
            auto findClosestProxOp = Operations::FindClosestProximity::Operation::get(l_surfGeom);
            auto projectOp = Operations::Project::Operation::get(l_surfGeom);
            auto projectFromOp = Operations::Project::Operation::get(l_tipGeom);

            for (; itfrom != l_tipGeom->end(); itfrom++) 
            {
                auto pfrom = createProximityOp(itfrom->element());
                if (pfrom == nullptr) continue;
                auto pdest = findClosestProxOp(pfrom, l_surfGeom.get(), projectOp, getFilterFunc());
                if (pdest != nullptr) {
                    pdest->normalize();

                    if (d_projective.getValue()) {
                        auto pfromProj = projectFromOp(pdest->getPosition(), itfrom->element()).prox;
                        if (pfromProj == nullptr) continue;
                        pfromProj->normalize();

                        collisionOutput.add(pfromProj, pdest);
                    }
                    else {
                        collisionOutput.add(pfrom, pdest);
                    }
                }
            }
        }
        else
        {
            insertionOutput.clear();

            auto itfrom = l_tipGeom->begin();
            auto createProximityOp = Operations::CreateCenterProximity::Operation::get(itfrom->getTypeInfo());
            auto pfrom = createProximityOp(itfrom->element());

            auto itfromVol = l_shaftGeom->begin(l_shaftGeom->getSize() - 2);
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
                auto findClosestProxOp_vol = Operations::FindClosestProximity::Operation::get(l_volGeom);
                auto projectOp_vol = Operations::Project::Operation::get(l_volGeom);
                auto projectFromOp_vol = Operations::Project::Operation::get(l_shaftGeom);
                auto pdestVol = findClosestProxOp_vol(pfrom, l_volGeom.get(), projectOp_vol, getFilterFunc());
                if (pdestVol)
                {
                    pdestVol->normalize();
                    m_couplingPts.push_back(pdestVol);
                    m_needlePts.push_back(m_needlePts.back());
                }
            }

            auto findClosestProxOp_needle = Operations::FindClosestProximity::Operation::get(l_shaftGeom);
            auto projectOp_needle = Operations::Project::Operation::get(l_shaftGeom);

            for(int i = 0 ; i < m_couplingPts.size(); i++)
            {
                auto pfromVol = findClosestProxOp_needle(m_couplingPts[i], l_shaftGeom.get(), projectOp_needle, getFilterFunc());
                m_needlePts[i] = pfromVol;
            }

            for(int i = 0 ; i < m_couplingPts.size(); i++)
                insertionOutput.add(m_needlePts[i], m_couplingPts[i]);
        }

        d_collisionOutput.endEdit();
        d_insertionOutput.endEdit();
    }

};

}


#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/collisionAlgorithm/operations/CreateCenterProximity.h>
#include <sofa/collisionAlgorithm/operations/FindClosestProximity.h>
#include <sofa/collisionAlgorithm/operations/Project.h>
#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>
#include <sofa/component/constraint/lagrangian/solver/ConstraintSolverImpl.h>
#include <sofa/component/statecontainer/MechanicalObject.h>

namespace sofa::collisionAlgorithm
{

class InsertionAlgorithm : public BaseAlgorithm
{
   public:
    SOFA_CLASS(InsertionAlgorithm, BaseAlgorithm);

    typedef core::behavior::MechanicalState<defaulttype::Vec3Types> MechStateTipType;
    typedef core::objectmodel::SingleLink<InsertionAlgorithm, BaseGeometry,
                                          BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK>
        GeomLink;
    typedef DetectionOutput<BaseProximity, BaseProximity> AlgorithmOutput;
    typedef component::constraint::lagrangian::solver::ConstraintSolverImpl ConstraintSolver;

    GeomLink l_tipGeom, l_surfGeom, l_shaftGeom, l_volGeom;
    Data<AlgorithmOutput> d_collisionOutput, d_insertionOutput;
    Data<bool> d_projective;
    Data<SReal> d_punctureForceThreshold, d_tipDistThreshold;
    ConstraintSolver::SPtr m_constraintSolver;
    std::vector<BaseProximity::SPtr> m_needlePts, m_couplingPts;
    Data<bool> d_drawCollision, d_drawPoints;
    Data<SReal> d_drawPointsScale;

    InsertionAlgorithm()
        : l_tipGeom(initLink("tipGeom", "Link to the geometry structure of the needle tip.")),
          l_surfGeom(
              initLink("surfGeom", "Link to the geometry of the surface punctured by the needle.")),
          l_shaftGeom(initLink("shaftGeom", "Link to the geometry structure of the needle shaft.")),
          l_volGeom(initLink("volGeom",
                             "Link to the geometry of volume wherein the needle is inserted.")),
          d_collisionOutput(initData(&d_collisionOutput, "collisionOutput",
                                     "Detected proximities during puncture.")),
          d_insertionOutput(initData(&d_insertionOutput, "insertionOutput",
                                     "Detected proximities during insertion.")),
          d_projective(initData(
              &d_projective, false, "projective",
              "Projection of closest detected proximity back onto the needle tip element.")),
          d_punctureForceThreshold(initData(&d_punctureForceThreshold,
                                            std::numeric_limits<double>::max(),
                                            "punctureForceThreshold",
                                            "Threshold for the force applied to the needle tip. "
                                            "Once exceeded, puncture is initiated.")),
          d_tipDistThreshold(initData(&d_tipDistThreshold, std::numeric_limits<double>::min(),
                                      "tipDistThreshold",
                                      "Threshold for the distance advanced by the needle tip since "
                                      "the last proximity detection. Once exceeded, a new "
                                      "proximity pair is added for the needle-volume coupling.")),
          m_constraintSolver(nullptr),
          m_needlePts(),
          m_couplingPts(),
          d_drawCollision(initData(&d_drawCollision, false, "drawcollision", "Draw collision.")),
          d_drawPoints(initData(&d_drawPoints, false, "drawPoints", "Draw detection outputs.")),
          d_drawPointsScale(initData(&d_drawPointsScale, 0.0005, "drawPointsScale",
                                     "Scale the drawing of detection output points."))
    {
    }

    void init() override
    {
        BaseAlgorithm::init();
        this->getContext()->get<ConstraintSolver>(m_constraintSolver);
        if (!m_constraintSolver)
            msg_warning("No constraint solver found in context. Insertion algorithm is disabled.");
    }

    void draw(const core::visual::VisualParams* vparams)
    {
        if (!vparams->displayFlags().getShowCollisionModels() && !d_drawCollision.getValue())
            return;
        vparams->drawTool()->disableLighting();

        const AlgorithmOutput& collisionOutput = d_collisionOutput.getValue();
        for (const auto& it : collisionOutput)
        {
            vparams->drawTool()->drawLine(it.first->getPosition(), it.second->getPosition(),
                                          type::RGBAColor(0, 1, 0, 1));
        }

        const AlgorithmOutput& insertionOutput = d_insertionOutput.getValue();
        for (const auto& it : insertionOutput)
        {
            vparams->drawTool()->drawSphere(it.first->getPosition(), d_drawPointsScale.getValue(),
                                            type::RGBAColor(1, 0, 1, 0.9));
            vparams->drawTool()->drawSphere(it.second->getPosition(), d_drawPointsScale.getValue(),
                                            type::RGBAColor(0, 0, 1, 0.9));
            vparams->drawTool()->drawLine(it.first->getPosition(), it.second->getPosition(),
                                          type::RGBAColor(1, 1, 0, 1));
        }
    }

    void doDetection()
    {
        if (!l_tipGeom || !l_surfGeom || !l_shaftGeom || !l_volGeom) return;

        auto& collisionOutput = *d_collisionOutput.beginEdit();
        auto& insertionOutput = *d_insertionOutput.beginEdit();

        if (insertionOutput.size() == 0)
        {
            const MechStateTipType::SPtr mstate = l_tipGeom->getContext()->get<MechStateTipType>();
            if (m_constraintSolver)
            {
                const auto& lambda =
                    m_constraintSolver->getLambda()[mstate.get()].read()->getValue();
                if (lambda[0].norm() > d_punctureForceThreshold.getValue())
                {
                    auto findClosestProxOnShaft =
                        Operations::FindClosestProximity::Operation::get(l_shaftGeom);
                    auto projectOnShaft = Operations::Project::Operation::get(l_shaftGeom);
                    for (const auto& dpair : collisionOutput)
                    {
                        // Reproject onto the needle to create an EdgeProximity - the
                        // EdgeNormalHandler in the Constraint classes will need this
                        const BaseProximity::SPtr shaftProx = findClosestProxOnShaft(
                            dpair.second, l_shaftGeom.get(), projectOnShaft, getFilterFunc());
                        m_needlePts.push_back(shaftProx);
                        m_couplingPts.push_back(dpair.second->copy());
                        insertionOutput.add(shaftProx, dpair.second->copy());
                    }
                    collisionOutput.clear();
                    return;
                }
            }

            collisionOutput.clear();

            ElementIterator::SPtr itTip = l_tipGeom->begin();
            auto createTipProximity =
                Operations::CreateCenterProximity::Operation::get(itTip->getTypeInfo());
            auto findClosestProxOnSurf =
                Operations::FindClosestProximity::Operation::get(l_surfGeom);
            auto projectOnSurf = Operations::Project::Operation::get(l_surfGeom);
            auto projectOnTip = Operations::Project::Operation::get(l_tipGeom);

            for (; itTip != l_tipGeom->end(); itTip++)
            {
                BaseProximity::SPtr tipProx = createTipProximity(itTip->element());
                if (!tipProx) continue;
                const BaseProximity::SPtr surfProx = findClosestProxOnSurf(
                    tipProx, l_surfGeom.get(), projectOnSurf, getFilterFunc());
                if (surfProx)
                {
                    surfProx->normalize();
                    if (d_projective.getValue())
                    {
                        tipProx = projectOnTip(surfProx->getPosition(), itTip->element()).prox;
                        if (!tipProx) continue;
                        tipProx->normalize();

                        collisionOutput.add(tipProx, surfProx);
                    }
                    else
                    {
                        collisionOutput.add(tipProx, surfProx);
                    }
                }
            }
        }
        else
        {
            insertionOutput.clear();

            ElementIterator::SPtr itTip = l_tipGeom->begin();
            auto createTipProximity =
                Operations::CreateCenterProximity::Operation::get(itTip->getTypeInfo());
            const BaseProximity::SPtr tipProx = createTipProximity(itTip->element());

            ElementIterator::SPtr itShaft = l_shaftGeom->begin(l_shaftGeom->getSize() - 2);
            auto createShaftProximity =
                Operations::CreateCenterProximity::Operation::get(itShaft->getTypeInfo());
            const BaseProximity::SPtr shaftProx = createShaftProximity(itShaft->element());
            const EdgeProximity::SPtr edgeProx = dynamic_pointer_cast<EdgeProximity>(shaftProx);
            const type::Vec3 normal = (edgeProx->element()->getP1()->getPosition() -
                                       edgeProx->element()->getP0()->getPosition())
                                          .normalized();
            const type::Vec3 ab = m_couplingPts.back()->getPosition() - tipProx->getPosition();
            const SReal dotProd = dot(ab, normal);
            if (dotProd > 0.0)
            {
                m_couplingPts.pop_back();
                m_needlePts.pop_back();
            }

            const SReal dist = ab.norm();
            if (dist > d_tipDistThreshold.getValue())
            {
                auto findClosestProxOnVol =
                    Operations::FindClosestProximity::Operation::get(l_volGeom);
                auto projectOnVol = Operations::Project::Operation::get(l_volGeom);
                const BaseProximity::SPtr volProx =
                    findClosestProxOnVol(tipProx, l_volGeom.get(), projectOnVol, getFilterFunc());
                if (volProx)
                {
                    volProx->normalize();
                    m_couplingPts.push_back(volProx);
                    m_needlePts.push_back(m_needlePts.back());
                }
            }

            auto findClosestProxOnShaft =
                Operations::FindClosestProximity::Operation::get(l_shaftGeom);
            auto projectOnShaft = Operations::Project::Operation::get(l_shaftGeom);

            for (int i = 0; i < m_couplingPts.size(); i++)
            {
                const BaseProximity::SPtr shaftProx = findClosestProxOnShaft(
                    m_couplingPts[i], l_shaftGeom.get(), projectOnShaft, getFilterFunc());
                m_needlePts[i] = shaftProx;
            }

            for (int i = 0; i < m_couplingPts.size(); i++)
                insertionOutput.add(m_needlePts[i], m_couplingPts[i]);
        }

        d_collisionOutput.endEdit();
        d_insertionOutput.endEdit();
    }
};

}  // namespace sofa::collisionAlgorithm

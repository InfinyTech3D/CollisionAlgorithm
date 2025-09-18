#pragma once

#include <CollisionAlgorithm/BaseAlgorithm.h>
#include <CollisionAlgorithm/BaseGeometry.h>
#include <CollisionAlgorithm/BaseOperation.h>
#include <CollisionAlgorithm/operations/ContainsPoint.h>
#include <CollisionAlgorithm/operations/CreateCenterProximity.h>
#include <CollisionAlgorithm/operations/FindClosestProximity.h>
#include <CollisionAlgorithm/operations/NeedleOperations.h>
#include <CollisionAlgorithm/operations/Project.h>
#include <CollisionAlgorithm/proximity/EdgeProximity.h>
#include <sofa/component/constraint/lagrangian/solver/ConstraintSolverImpl.h>
#include <sofa/component/statecontainer/MechanicalObject.h>

namespace sofa::collisionalgorithm
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
    Data<bool> d_projective, d_enablePuncture, d_enableInsertion;
    Data<SReal> d_punctureForceThreshold, d_tipDistThreshold;
    ConstraintSolver::SPtr m_constraintSolver;
    std::vector<BaseProximity::SPtr> m_couplingPts;
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
          d_enablePuncture(
              initData(&d_enablePuncture, true, "enablePuncture", "Enable puncture algorithm.")),
          d_enableInsertion(
              initData(&d_enableInsertion, true, "enableInsertion", "Enable insertion algorithm.")),
          d_punctureForceThreshold(initData(&d_punctureForceThreshold, -1_sreal,
                                            "punctureForceThreshold",
                                            "Threshold for the force applied to the needle tip. "
                                            "Once exceeded, puncture is initiated.")),
          d_tipDistThreshold(initData(&d_tipDistThreshold, -1_sreal, "tipDistThreshold",
                                      "Threshold for the distance advanced by the needle tip since "
                                      "the last proximity detection. Once exceeded, a new "
                                      "proximity pair is added for the needle-volume coupling.")),
          m_constraintSolver(nullptr),
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
        msg_warning_when(!m_constraintSolver)
            << "No constraint solver found in context. Insertion algorithm is disabled.";

        if (d_punctureForceThreshold.getValue() < 0)
        {
            msg_warning() << d_punctureForceThreshold.getName() +
                                 " parameter not defined or set to negative value." msgendl
                          << "Puncture will not function properly; provide a positive value";
            d_punctureForceThreshold.setValue(std::numeric_limits<double>::max());
        }

        if (d_tipDistThreshold.getValue() < 0)
        {
            msg_warning() << d_tipDistThreshold.getName() +
                                 " parameter not defined or set to negative value." msgendl
                          << "Needle-volume coupling is disabled; provide a positive value";
            d_tipDistThreshold.setValue(std::numeric_limits<double>::max());
        }
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

        insertionOutput.clear();
        collisionOutput.clear();

        if (m_couplingPts.empty())
        {
            // Operations on surface geometry
            auto findClosestProxOnSurf =
                Operations::FindClosestProximity::Operation::get(l_surfGeom);
            auto projectOnSurf = Operations::Project::Operation::get(l_surfGeom);

            // Puncture sequence
            if (d_enablePuncture.getValue())
            {
                auto createTipProximity =
                    Operations::CreateCenterProximity::Operation::get(l_tipGeom->getTypeInfo());
                auto projectOnTip = Operations::Project::Operation::get(l_tipGeom);
    
                const SReal punctureForceThreshold = d_punctureForceThreshold.getValue();
                for (auto itTip = l_tipGeom->begin(); itTip != l_tipGeom->end(); itTip++)
                {
                    BaseProximity::SPtr tipProx = createTipProximity(itTip->element());
                    if (!tipProx) continue;
                    const BaseProximity::SPtr surfProx = findClosestProxOnSurf(
                        tipProx, l_surfGeom.get(), projectOnSurf, getFilterFunc());
                    if (surfProx)
                    {
                        surfProx->normalize();
    
                        // Check whether puncture is happening - if so, create coupling point ...
                        if (m_constraintSolver)
                        {
                            const MechStateTipType::SPtr mstate =
                                l_tipGeom->getContext()->get<MechStateTipType>();
                            const auto& lambda =
                                m_constraintSolver->getLambda()[mstate.get()].read()->getValue();
                            SReal norm{0_sreal};
                            for (const auto& l : lambda)
                            {
                                norm += l.norm();
                            }
                            if (norm > punctureForceThreshold)
                            {
                                m_couplingPts.push_back(surfProx);
                                continue;
                            }
                        }
    
                        // ... if not, create a proximity pair for the tip-surface collision
                        collisionOutput.add(tipProx, surfProx);
                    }
                }
            }

            // Shaft collision sequence - Disable if coupling points have been added
            if (m_couplingPts.empty())
            {
                auto createShaftProximity =
                    Operations::CreateCenterProximity::Operation::get(l_shaftGeom->getTypeInfo());
                auto projectOnShaft = Operations::Project::Operation::get(l_shaftGeom);
                for (auto itShaft = l_shaftGeom->begin(); itShaft != l_shaftGeom->end(); itShaft++)
                {
                    BaseProximity::SPtr shaftProx = createShaftProximity(itShaft->element());
                    if (!shaftProx) continue;
                    const BaseProximity::SPtr surfProx = findClosestProxOnSurf(
                        shaftProx, l_surfGeom.get(), projectOnSurf, getFilterFunc());
                    if (surfProx)
                    {
                        surfProx->normalize();

                        if (d_projective.getValue())
                        {
                            shaftProx =
                                projectOnShaft(surfProx->getPosition(), itShaft->element()).prox;
                            if (!shaftProx) continue;
                            shaftProx->normalize();
                        }
                        collisionOutput.add(shaftProx, surfProx);
                    }
                }
            }
        }
        else
        {
            // Insertion sequence
            if (!d_enableInsertion.getValue()) return;

            ElementIterator::SPtr itTip = l_tipGeom->begin();
            auto createTipProximity =
                Operations::CreateCenterProximity::Operation::get(itTip->getTypeInfo());
            const BaseProximity::SPtr tipProx = createTipProximity(itTip->element());
            if (!tipProx) return;

            // Remove coupling points that are ahead of the tip in the insertion direction
            ElementIterator::SPtr itShaft = l_shaftGeom->begin(l_shaftGeom->getSize() - 2);
            auto prunePointsAheadOfTip = 
                Operations::Needle::PrunePointsAheadOfTip::get(itShaft->getTypeInfo());
            prunePointsAheadOfTip(m_couplingPts, itShaft->element());

            if (m_couplingPts.empty()) return;

            type::Vec3 lastCP = m_couplingPts.back()->getPosition();
            const SReal tipDistThreshold = this->d_tipDistThreshold.getValue();

            // Vector from tip to last coupling point; used for distance and directional checks
            const type::Vec3 tipToLastCP = lastCP - tipProx->getPosition();

            // Only add a new coupling point if the needle tip has advanced far enough
            if (tipToLastCP.norm() > tipDistThreshold)
            {
                // Prepare the operations before entering the loop
                auto createShaftProximity =
                    Operations::CreateCenterProximity::Operation::get(l_shaftGeom->getTypeInfo());
                auto projectOnShaft = Operations::Project::Operation::get(l_shaftGeom);
                auto findClosestProxOnVol =
                    Operations::FindClosestProximity::Operation::get(l_volGeom);
                auto projectOnVol = Operations::Project::Operation::get(l_volGeom);
                auto containsPointInVol =
                    Operations::ContainsPointInProximity::Operation::get(l_volGeom);

                // Iterate over shaft segments to find which one contains the next candidate CP
                for (auto itShaft = l_shaftGeom->begin(); itShaft != l_shaftGeom->end(); itShaft++)
                {
                    BaseProximity::SPtr shaftProx = createShaftProximity(itShaft->element());
                    if (!shaftProx) continue;

                    const EdgeProximity::SPtr edgeProx =
                        dynamic_pointer_cast<EdgeProximity>(shaftProx);
                    if (!edgeProx) continue;

                    const type::Vec3 p0 = edgeProx->element()->getP0()->getPosition();
                    const type::Vec3 p1 = edgeProx->element()->getP1()->getPosition();
                    const type::Vec3 shaftEdgeDir = (p1 - p0).normalized();
                    const type::Vec3 lastCPToP1 = p1 - lastCP;

                    // Skip if last CP lies after edge end point
                    if (dot(shaftEdgeDir, lastCPToP1) < 0_sreal) continue;

                    const int numCPs = floor(lastCPToP1.norm() / tipDistThreshold);

                    for (int idCP = 0; idCP < numCPs; idCP++)
                    {
                        // Candidate coupling point along shaft segment
                        const type::Vec3 candidateCP = lastCP + tipDistThreshold * shaftEdgeDir;

                        // Project candidate CP onto the edge element and compute scalar coordinate
                        // along segment
                        const SReal edgeSegmentLength = (p1 - p0).norm();
                        const type::Vec3 p0ToCandidateCP = candidateCP - p0;
                        const SReal projPtOnEdge = dot(p0ToCandidateCP, shaftEdgeDir);

                        // Skip if candidate CP is outside current edge segment
                        if (projPtOnEdge < 0_sreal || projPtOnEdge > edgeSegmentLength) break;

                        // Project candidate CP onto shaft geometry ...
                        shaftProx = projectOnShaft(candidateCP, itShaft->element()).prox;
                        if (!shaftProx) continue;

                        // ... then find nearest volume proximity
                        const BaseProximity::SPtr volProx = findClosestProxOnVol(
                            shaftProx, l_volGeom.get(), projectOnVol, getFilterFunc());
                        if (!volProx) continue;

                        // Proximity can be detected before the tip enters the tetra (e.g. near a
                        // boundary face) Only accept proximities if the tip is inside the tetra
                        // during insertion
                        if (containsPointInVol(shaftProx->getPosition(), volProx))
                        {
                            volProx->normalize();
                            m_couplingPts.push_back(volProx);
                            lastCP = volProx->getPosition();
                        }
                    }
                }
            }
            else  // Don't bother with removing the point that was just added
            {
                // Remove coupling points that are ahead of the tip in the insertion direction
                ElementIterator::SPtr itShaft = l_shaftGeom->begin(l_shaftGeom->getSize() - 2);
                auto prunePointsAheadOfTip =
                    Operations::Needle::PrunePointsAheadOfTip::get(itShaft->getTypeInfo());
                prunePointsAheadOfTip(m_couplingPts, itShaft->element());
            }
        }

        if (d_enableInsertion.getValue() && !m_couplingPts.empty())
        {
            // Reprojection on shaft geometry sequence
            auto findClosestProxOnShaft =
                Operations::FindClosestProximity::Operation::get(l_shaftGeom);
            auto projectOnShaft = Operations::Project::Operation::get(l_shaftGeom);
            for (int i = 0; i < m_couplingPts.size(); i++)
            {
                const BaseProximity::SPtr shaftProx = findClosestProxOnShaft(
                    m_couplingPts[i], l_shaftGeom.get(), projectOnShaft, getFilterFunc());
                if (!shaftProx) continue;
                shaftProx->normalize();
                insertionOutput.add(shaftProx, m_couplingPts[i]);
            }
            // This is a final-frontier check: If there are coupling points stored, but the
            // findClosestProxOnShaf operation yields no proximities on the shaft, it could be
            // because the needle has exited abruptly. Thus, we clear the coupling points.
            if (insertionOutput.size() == 0) m_couplingPts.clear();
        }

        d_collisionOutput.endEdit();
        d_insertionOutput.endEdit();
    }
};

}  // namespace sofa::collisionalgorithm

#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/collisionAlgorithm/operations/CreateCenterProximity.h>
#include <sofa/collisionAlgorithm/operations/FindClosestProximity.h>
#include <sofa/collisionAlgorithm/operations/Project.h>
#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>
#include <sofa/collisionAlgorithm/proximity/TetrahedronProximity.h>
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
    std::vector<BaseProximity::SPtr> m_CPs;
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
          d_punctureForceThreshold(initData(&d_punctureForceThreshold, -1_sreal,
                                            "punctureForceThreshold",
                                            "Threshold for the force applied to the needle tip. "
                                            "Once exceeded, puncture is initiated.")),
          d_tipDistThreshold(initData(&d_tipDistThreshold, -1_sreal, "tipDistThreshold",
                                      "Threshold for the distance advanced by the needle tip since "
                                      "the last proximity detection. Once exceeded, a new "
                                      "proximity pair is added for the needle-volume coupling.")),
          m_constraintSolver(nullptr),
          m_CPs(),
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

        if (m_CPs.empty())
        {
            // 1. Puncture algorithm
            auto createTipProximity =
                Operations::CreateCenterProximity::Operation::get(l_tipGeom->getTypeInfo());
            auto findClosestProxOnSurf =
                Operations::FindClosestProximity::Operation::get(l_surfGeom);
            auto projectOnSurf = Operations::Project::Operation::get(l_surfGeom);
            auto projectOnTip = Operations::Project::Operation::get(l_tipGeom);

            const bool isProjective = d_projective.getValue();
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

                    // 1.1 Check whether puncture is happening - if so, create coupling point
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
                            m_CPs.push_back(surfProx);
                            continue;
                        }
                    }

                    // 1.2 If not, create a proximity pair for the tip-surface collision
                    if (isProjective)
                    {
                        tipProx = projectOnTip(surfProx->getPosition(), itTip->element()).prox;
                        if (!tipProx) continue;
                        tipProx->normalize();
                    }
                    collisionOutput.add(tipProx, surfProx);
                }
            }

            // 1.3 Collision with the shaft geometry
            if (m_CPs.empty())
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

                        // 1.2 If not, create a proximity pair for the tip-surface collision
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
            // 2. Needle insertion algorithm
            ElementIterator::SPtr itTip = l_tipGeom->begin();
            auto createTipProximity =
                Operations::CreateCenterProximity::Operation::get(itTip->getTypeInfo());
            const BaseProximity::SPtr tipProx = createTipProximity(itTip->element());
            if (!tipProx) return;

            type::Vec3 lastCP = m_CPs.back()->getPosition();
            const SReal tipDistThreshold = this->d_tipDistThreshold.getValue();

            // Vector from tip to last coupling point; used for distance and directional checks
            const type::Vec3 tipToLastCP = lastCP - tipProx->getPosition();

            // Only add a new coupling point if the needle tip has advanced far enough
            if (tipToLastCP.norm() > tipDistThreshold)
            {
                auto createShaftProximity =
                    Operations::CreateCenterProximity::Operation::get(l_shaftGeom->getTypeInfo());
                auto projectOnShaft = Operations::Project::Operation::get(l_shaftGeom);
                auto findClosestProxOnVol =
                    Operations::FindClosestProximity::Operation::get(l_volGeom);
                auto projectOnVol = Operations::Project::Operation::get(l_volGeom);

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

                    for(int idCP = 0 ; idCP < numCPs ; idCP++)
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

                        // Project candidate CP onto shaft geometry and then find nearest volume
                        // proximity
                        shaftProx = projectOnShaft(candidateCP, itShaft->element()).prox;
                        if (!shaftProx) continue;
                        const BaseProximity::SPtr volProx = findClosestProxOnVol(
                            shaftProx, l_volGeom.get(), projectOnVol, getFilterFunc());
                        if (!volProx) continue;

                        TetrahedronProximity::SPtr tetProx =
                            dynamic_pointer_cast<TetrahedronProximity>(volProx);
                        if (!tetProx) continue;

                        double f0(tetProx->f0()), f1(tetProx->f1()), f2(tetProx->f2()),
                            f3(tetProx->f3());
                        bool isInTetra = toolbox::TetrahedronToolBox::isInTetra(
                            shaftProx->getPosition(), tetProx->element()->getTetrahedronInfo(), f0,
                            f1, f2, f3);

                        // Ensure candidate CP lies inside tetrahedron
                        if (isInTetra)
                        {
                            volProx->normalize();
                            m_CPs.push_back(volProx);
                            lastCP = volProx->getPosition();
                        }
                    }
                }
            }
            else  // Don't bother with removing the point that was just added
            {
                // 2.2. Check whether coupling point should be removed
                ElementIterator::SPtr itShaft = l_shaftGeom->begin(l_shaftGeom->getSize() - 2);
                auto createShaftProximity =
                    Operations::CreateCenterProximity::Operation::get(itShaft->getTypeInfo());
                const BaseProximity::SPtr shaftProx = createShaftProximity(itShaft->element());
                if (shaftProx)
                {
                    const EdgeProximity::SPtr edgeProx =
                        dynamic_pointer_cast<EdgeProximity>(shaftProx);
                    if (edgeProx)
                    {
                        const type::Vec3 normal = (edgeProx->element()->getP1()->getPosition() -
                                                   edgeProx->element()->getP0()->getPosition())
                                                      .normalized();
                        // If the (last) coupling point lies ahead of the tip (positive dot
                        // product), the needle is retreating. Thus, that point is removed.
                        if (dot(tipToLastCP, normal) > 0_sreal)
                        {
                            m_CPs.pop_back();
                        }
                    }
                    else
                    {
                        msg_warning() << "shaftGeom: " << l_shaftGeom->getName()
                                      << " is not an EdgeGeometry. Point removal is disabled";
                    }
                }
                else
                {
                    msg_warning() << "Cannot create proximity from shaftGeom: "
                                  << l_shaftGeom->getName() << " - point removal is disabled";
                }
            }
        }

        if (!m_CPs.empty())
        {
            // 3. Re-project proximities on the shaft geometry
            auto findClosestProxOnShaft =
                Operations::FindClosestProximity::Operation::get(l_shaftGeom);
            auto projectOnShaft = Operations::Project::Operation::get(l_shaftGeom);
            for (int i = 0; i < m_CPs.size(); i++)
            {
                const BaseProximity::SPtr shaftProx = findClosestProxOnShaft(
                    m_CPs[i], l_shaftGeom.get(), projectOnShaft, getFilterFunc());
                if (!shaftProx) continue;
                shaftProx->normalize();
                insertionOutput.add(shaftProx, m_CPs[i]);
            }
            // This is a final-frontier check: If there are coupling points stored, but the
            // findClosestProxOnShaf operation yields no proximities on the shaft, it could be
            // because the needle has exited abruptly. Thus, we clear the coupling points.
            if (insertionOutput.size() == 0) m_CPs.clear();
        }

        d_collisionOutput.endEdit();
        d_insertionOutput.endEdit();
    }
};

}  // namespace sofa::collisionAlgorithm

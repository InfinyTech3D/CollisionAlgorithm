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

class SOFA_COLLISIONALGORITHM_API InsertionAlgorithm : public BaseAlgorithm
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
    Data<bool> d_projective, d_enablePuncture, d_enableInsertion, d_enableShaftCollision;
    Data<SReal> d_punctureForceThreshold, d_tipDistThreshold;
    ConstraintSolver::SPtr m_constraintSolver;
    std::vector<BaseProximity::SPtr> m_couplingPts;
    Data<bool> d_drawCollision, d_drawPoints;
    Data<SReal> d_drawPointsScale;

    InsertionAlgorithm();
    void init() override;
    void draw(const core::visual::VisualParams* vparams) override;

    /// Performs a proximity detection step during needle puncture and insertion.
    /// Detection outputs are used to create collisions and needle-tissue coupling.
    /// Handles puncture, shaft collisions and insertion into tissue phases.
    void doDetection() override;
};

}  // namespace sofa::collisionalgorithm

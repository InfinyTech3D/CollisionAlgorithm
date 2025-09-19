#pragma once

#include <CollisionAlgorithm/config.h>
#include <CollisionAlgorithm/elements/PointElement.h>

namespace sofa::collisionalgorithm {

class SOFA_COLLISIONALGORITHM_API PointProximity : public BaseProximity {
public:

    typedef std::shared_ptr<PointProximity> SPtr;

    PointProximity(PointElement::SPtr elmt)
    : m_elmt(elmt) {}

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vec3> & dir, double fact, Index constraintId) const override {
        m_elmt->getP0()->buildJacobianConstraint(cId,dir,fact,constraintId);
    }

    /// return proximiy position in a Vec3
    sofa::type::Vec3 getPosition(core::VecCoordId v = core::vec_id::write_access::position) const {
        return m_elmt->getP0()->getPosition(v);
    }

    sofa::type::Vec3 getVelocity(core::VecDerivId v = core::vec_id::write_access::velocity) const {
        return m_elmt->getP0()->getVelocity(v);
    }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, Index cid_global, Index cid_local, const sofa::linearalgebra::BaseVector* lambda) const override {
        m_elmt->getP0()->storeLambda(cParams,res,cid_global,cid_local,lambda);
    }

    PointElement::SPtr element() { return m_elmt; }

    static BaseProximity::SPtr create(const PointElement::SPtr & p) {
        return p->getP0();
    }

	virtual BaseProximity::SPtr copy() override
	{
		return PointProximity::create(m_elmt);
	}

    const std::type_info& getTypeInfo() const override { return typeid(PointProximity); }

    bool isNormalized() const override { return true; }

    void normalize() override {}

    double * getBaryCoord() override {
        return nullptr;
    }

protected:
    PointElement::SPtr m_elmt;
};

}


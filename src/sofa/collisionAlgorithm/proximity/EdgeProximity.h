#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>

namespace sofa::collisionAlgorithm {

class EdgeProximity : public BaseProximity {
public:

    typedef std::shared_ptr<EdgeProximity> SPtr;

    EdgeProximity(EdgeElement::SPtr elmt,double f0,double f1)
    : m_elmt(elmt), m_f0(f0), m_f1(f1) {}

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vec3> & dir, double fact, Index constraintId) const override {
        sofa::type::vector<sofa::type::Vec3> N1;
        for (unsigned i=0;i<dir.size();i++) N1.push_back(dir[i]*m_f0);
        m_elmt->getP0()->buildJacobianConstraint(cId,N1,fact,constraintId);

        sofa::type::vector<sofa::type::Vec3> N2;
        for (unsigned i=0;i<dir.size();i++) N2.push_back(dir[i]*m_f1);
        m_elmt->getP1()->buildJacobianConstraint(cId,N2,fact,constraintId);
    }


    /// return proximiy position in a Vec3
    sofa::type::Vec3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const override {
        return m_elmt->getP0()->getPosition(v) * m_f0 +
               m_elmt->getP1()->getPosition(v) * m_f1;
    }

    /// return proximiy velocity in a Vec3
    sofa::type::Vec3 getVelocity(core::VecDerivId v = core::VecDerivId::velocity()) const override {
        return m_elmt->getP0()->getVelocity(v) * m_f0 +
               m_elmt->getP1()->getVelocity(v) * m_f1;
    }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, Index cid_global, Index cid_local, const sofa::linearalgebra::BaseVector* lambda) const override {
        m_elmt->getP0()->storeLambda(cParams,res,cid_global,cid_local,lambda);
        m_elmt->getP1()->storeLambda(cParams,res,cid_global,cid_local,lambda);
    }

    EdgeElement::SPtr element() { return m_elmt; }

    static EdgeProximity::SPtr create(EdgeElement::SPtr sptr, double f0,double f1) {
        return EdgeProximity::SPtr(new EdgeProximity(sptr,f0,f1));
    }

	virtual BaseProximity::SPtr copy() override
	{
		return EdgeProximity::create(m_elmt,m_f0,m_f1);
	}

    const std::type_info& getTypeInfo() const override { return typeid(EdgeProximity); }

    double f0() const { return m_f0; }

    double f1() const { return m_f1; }

    double * getBaryCoord() override {
        static double baryCoord[2] = {m_f0,m_f1};
        return baryCoord;
    }

    bool isNormalized() const override {
//        if (m_f0+m_f1 != 1.0) return false;

        return m_f0>=0 && m_f0<=1 &&
               m_f1>=0 && m_f1<=1;
    }

    void normalize() override {
        if (m_f1<0.0) m_f1 = 0.0;
        else if (m_f1>1.0) m_f1 = 1.0;

        m_f0 = 1.0-m_f1;
    }

protected:
    EdgeElement::SPtr m_elmt;
    double m_f0,m_f1;
};

}

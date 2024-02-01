#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>
#include <sofa/collisionAlgorithm/toolbox/TriangleToolBox.h>

namespace sofa::collisionAlgorithm {

class TriangleProximity : public BaseProximity {
public:

    typedef std::shared_ptr<TriangleProximity> SPtr;

    TriangleProximity(const TriangleElement::SPtr & elmt, double f0,double f1,double f2)
    : m_elmt(elmt), m_f0(f0), m_f1(f1), m_f2(f2){}


    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vec3> & dir, double fact, Index constraintId) const override {
        sofa::type::vector<sofa::type::Vec3> N0;
        sofa::type::vector<sofa::type::Vec3> N1;
        sofa::type::vector<sofa::type::Vec3> N2;

        for (unsigned i=0;i<dir.size();i++) {
            N0.push_back(dir[i]*m_f0);
            N1.push_back(dir[i]*m_f1);
            N2.push_back(dir[i]*m_f2);
        }

        m_elmt->getP0()->buildJacobianConstraint(cId,N0,fact,constraintId);
        m_elmt->getP1()->buildJacobianConstraint(cId,N1,fact,constraintId);
        m_elmt->getP2()->buildJacobianConstraint(cId,N2,fact,constraintId);

    }

    /// return proximiy position in a Vec3
    sofa::type::Vec3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
        return m_elmt->getP0()->getPosition(v) * m_f0 +
               m_elmt->getP1()->getPosition(v) * m_f1 +
               m_elmt->getP2()->getPosition(v) * m_f2;
    }

    sofa::type::Vec3 getVelocity(core::VecDerivId v = core::VecDerivId::velocity()) const {
        return m_elmt->getP0()->getVelocity(v) * m_f0 +
               m_elmt->getP1()->getVelocity(v) * m_f1 +
               m_elmt->getP2()->getVelocity(v) * m_f2;
    }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, Index cid_global, Index cid_local, const sofa::linearalgebra::BaseVector* lambda) const override {
        m_elmt->getP0()->storeLambda(cParams,res,cid_global,cid_local,lambda);
        m_elmt->getP1()->storeLambda(cParams,res,cid_global,cid_local,lambda);
        m_elmt->getP2()->storeLambda(cParams,res,cid_global,cid_local,lambda);
    }

    TriangleElement::SPtr element() { return m_elmt; }

    double f0() { return m_f0; }

    double f1() { return m_f1; }

    double f2() { return m_f2; }

    double * getBaryCoord() override {
        static double baryCoord[3] = {m_f0,m_f1,m_f2};
        return baryCoord;
    }

    static TriangleProximity::SPtr create(const TriangleElement::SPtr & tri, double f0,double f1,double f2) {
        return TriangleProximity::SPtr(new TriangleProximity(tri,f0,f1,f2));
    }

	virtual BaseProximity::SPtr copy() override
	{
		return TriangleProximity::create(m_elmt,m_f0,m_f1,m_f2);
	}

    const std::type_info& getTypeInfo() const override { return typeid(TriangleProximity); }

    bool isNormalized() const override {
//        if (m_f0+m_f1+m_f2 != 1.0) return false;

        return m_f0>=0 && m_f0<=1 &&
               m_f1>=0 && m_f1<=1 &&
               m_f2>=0 && m_f2<=1;
    }

    void normalize() override {
        toolbox::TriangleToolBox::normalize(m_elmt->getP0()->getPosition(),
                                            m_elmt->getP1()->getPosition(),
                                            m_elmt->getP2()->getPosition(),
                                            m_f0,m_f1,m_f2);
    }

protected:
    TriangleElement::SPtr m_elmt;
    double m_f0,m_f1,m_f2;
};

}

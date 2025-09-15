#pragma once

#include <CollisionAlgorithm/elements/TetrahedronElement.h>
#include <CollisionAlgorithm/toolbox/TriangleToolBox.h>
#include <CollisionAlgorithm/toolbox/TetrahedronToolBox.h>

namespace sofa::collisionalgorithm {

class TetrahedronProximity : public BaseProximity {
public:

    typedef std::shared_ptr<TetrahedronProximity> SPtr;

    TetrahedronProximity(const TetrahedronElement::SPtr & elmt, double f0,double f1,double f2,double f3)
    : m_elmt(elmt), m_f0(f0), m_f1(f1), m_f2(f2), m_f3(f3){}


    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vec3> & dir, double fact, Index constraintId) const override {
        sofa::type::vector<sofa::type::Vec3> N0;
        sofa::type::vector<sofa::type::Vec3> N1;
        sofa::type::vector<sofa::type::Vec3> N2;
        sofa::type::vector<sofa::type::Vec3> N3;

        for (unsigned i=0;i<dir.size();i++) {
            N0.push_back(dir[i]*m_f0);
            N1.push_back(dir[i]*m_f1);
            N2.push_back(dir[i]*m_f2);
            N3.push_back(dir[i]*m_f3);
        }
        m_elmt->getP0()->buildJacobianConstraint(cId,N0,fact,constraintId);
        m_elmt->getP1()->buildJacobianConstraint(cId,N1,fact,constraintId);
        m_elmt->getP2()->buildJacobianConstraint(cId,N2,fact,constraintId);
        m_elmt->getP3()->buildJacobianConstraint(cId,N3,fact,constraintId);

    }

    /// return proximiy position in a Vec3
    sofa::type::Vec3 getPosition(core::VecCoordId v = core::vec_id::write_access::position) const override {
        return m_elmt->getP0()->getPosition(v) * m_f0 +
               m_elmt->getP1()->getPosition(v) * m_f1 +
               m_elmt->getP2()->getPosition(v) * m_f2 +
               m_elmt->getP3()->getPosition(v) * m_f3;
    }

    sofa::type::Vec3 getVelocity(core::VecDerivId v = core::vec_id::write_access::velocity) const override {
        return m_elmt->getP0()->getVelocity(v) * m_f0 +
               m_elmt->getP1()->getVelocity(v) * m_f1 +
               m_elmt->getP2()->getVelocity(v) * m_f2 +
               m_elmt->getP3()->getVelocity(v) * m_f3;
    }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, Index cid_global, Index cid_local, const sofa::linearalgebra::BaseVector* lambda) const override {
        m_elmt->getP0()->storeLambda(cParams,res,cid_global,cid_local,lambda);
        m_elmt->getP1()->storeLambda(cParams,res,cid_global,cid_local,lambda);
        m_elmt->getP2()->storeLambda(cParams,res,cid_global,cid_local,lambda);
        m_elmt->getP3()->storeLambda(cParams,res,cid_global,cid_local,lambda);
    }

    TetrahedronElement::SPtr element() { return m_elmt; }


    static TetrahedronProximity::SPtr create(const TetrahedronElement::SPtr & tetra, double f0,double f1,double f2,double f3) {
        return TetrahedronProximity::SPtr(new TetrahedronProximity(tetra,f0,f1,f2,f3));
    }

	virtual BaseProximity::SPtr copy() override
	{
		return TetrahedronProximity::create(m_elmt,m_f0,m_f1,m_f2,m_f3);
	}

    const std::type_info& getTypeInfo() const override { return typeid(TetrahedronProximity); }

    double f0() { return m_f0; }

    double f1() { return m_f1; }

    double f2() { return m_f2; }

    double f3() { return m_f3; }

    double * getBaryCoord() override {
        static double baryCoord[4] = {m_f0,m_f1,m_f2,m_f3};
        return baryCoord;
    }

    bool isNormalized() const override {
//        if (m_f0+m_f1+m_f2+m_f3 != 1.0) return false;

        return m_f0>=0 && m_f0<=1 &&
               m_f1>=0 && m_f1<=1 &&
               m_f2>=0 && m_f2<=1 &&
               m_f3>=0 && m_f3<=1;
    }

    void normalize() override {
        toolbox::TetrahedronToolBox::normalize(m_elmt->getP0()->getPosition(),
                                               m_elmt->getP1()->getPosition(),
                                               m_elmt->getP2()->getPosition(),
                                               m_elmt->getP3()->getPosition(),
                                               m_f0,m_f1,m_f2,m_f3);
    }

protected:
    TetrahedronElement::SPtr m_elmt;
    double m_f0,m_f1,m_f2,m_f3;
};

}

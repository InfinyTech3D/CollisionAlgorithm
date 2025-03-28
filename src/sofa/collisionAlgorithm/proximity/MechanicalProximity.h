#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>

namespace sofa::collisionAlgorithm {

template<class DataTypes>
class MechanicalProximity : public BaseProximity {
public:

    typedef std::shared_ptr<MechanicalProximity<DataTypes> > SPtr;

    typedef sofa::core::behavior::MechanicalState<DataTypes> State;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename DataTypes::Deriv Deriv1;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::objectmodel::Data< VecDeriv >        DataVecDeriv;
    typedef core::objectmodel::Data< MatrixDeriv >     DataMatrixDeriv;


    MechanicalProximity(TBaseGeometry<DataTypes> * geo, unsigned pid)
    : m_geometry(geo)
    , m_pid(pid) {}


    BaseGeometry * getGeometry() const {
        return m_geometry;
    }

    TBaseGeometry<DataTypes> * getTGeometry() const {
        return m_geometry;
    }

    void addContributions(MatrixDerivRowIterator & c_it, const sofa::type::Vec3 & N,double fact) const {
        c_it.addCol(m_pid, N * fact);
    }

    /// return proximiy position in a Vec3
    sofa::type::Vec3 getPosition(core::VecCoordId v = core::vec_id::write_access::position) const {
        return m_geometry->getPosition(m_pid,v);
    }

    sofa::type::Vec3 getVelocity(core::VecDerivId v = core::vec_id::write_access::velocity) const {
        return m_geometry->getVelocity(m_pid,v);
    }

    unsigned getPId() const {
        return m_pid;
    }

    const std::type_info& getTypeInfo() const override { return typeid(MechanicalProximity<DataTypes>); }

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vec3> & dir, double fact, Index constraintId) const override {
        DataMatrixDeriv & c1_d = *cId[m_geometry->getState()].write();
        MatrixDeriv & c1 = *c1_d.beginEdit();

        for (Index j=0;j<dir.size();j++) {
            MatrixDerivRowIterator c_it = c1.writeLine(constraintId+j);
            addContributions(c_it,dir[j],fact);
        }

        c1_d.endEdit();
    }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId resId, Index cid_global, Index cid_local, const sofa::linearalgebra::BaseVector* lambda) const override {
        auto res = sofa::helper::getWriteAccessor(*resId[m_geometry->getState()].write());
        const typename DataTypes::MatrixDeriv& j = cParams->readJ(m_geometry->getState())->getValue();
        auto rowIt = j.readLine(cid_global+cid_local);
        const double f = lambda->element(cid_global+cid_local);
        for (auto colIt = rowIt.begin(), colItEnd = rowIt.end(); colIt != colItEnd; ++colIt)
        {
            res[colIt.index()] += colIt.val() * f;
        }
    }

	virtual BaseProximity::SPtr copy() override
	{
		return SPtr(new MechanicalProximity(m_geometry,m_pid));
	}

    bool isNormalized() const override { return true; }

    void normalize() override {}

    double * getBaryCoord() override {
        return nullptr;
    }

protected:
    TBaseGeometry<DataTypes> * m_geometry;
    unsigned m_pid;
};

}


#pragma once

#include <sofa/core/VecId.h>
#include <sofa/core/MultiVecId.h>
#include <sofa/linearalgebra/BaseVector.h>
#include <sofa/core/ConstraintParams.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/topology/Topology.h>
#include <sofa/type/Vec.h>

namespace sofa::collisionalgorithm {

/*!
 * \brief The BaseProximity class is the basic abstract proximity class
 */
class BaseElement;

class BaseBaseProximity  {
public:
    typedef std::shared_ptr<BaseBaseProximity> SPtr;

    /// return proximiy position in a Vec3
    virtual sofa::type::Vec3 getPosition(core::VecCoordId v = core::vec_id::write_access::position) const = 0;

    virtual sofa::type::Vec3 getVelocity(core::VecDerivId v = core::vec_id::write_access::velocity) const = 0;

    virtual void buildJacobianConstraint(core::MultiMatrixDerivId , const sofa::type::vector<sofa::type::Vec3> & , double , Index ) const = 0;

    virtual void storeLambda(const core::ConstraintParams* , core::MultiVecDerivId , Index , Index , const sofa::linearalgebra::BaseVector* ) const = 0;

    const std::type_info & getTypeInfo() { return typeid(*this); }
};

class BaseProximity : public virtual BaseBaseProximity {
public:
    typedef std::shared_ptr<BaseProximity> SPtr;

    virtual ~BaseProximity() = default;

    virtual const std::type_info& getTypeInfo() const = 0;

    template<class PROXIMITY,class... ARGS>
    static inline typename PROXIMITY::SPtr create(ARGS... args) {
        return typename PROXIMITY::SPtr(new PROXIMITY(args...));
    }

	virtual SPtr copy() = 0;

    virtual bool isNormalized() const = 0;

    virtual void normalize() = 0;

    virtual double * getBaryCoord() = 0;

    virtual sofa::type::Vec3 getPosition(core::VecCoordId = core::vec_id::write_access::position) const override { return  sofa::type::Vec3{}; }

    virtual sofa::type::Vec3 getVelocity(core::VecDerivId = core::vec_id::write_access::velocity) const override { return  sofa::type::Vec3{}; }

    virtual void buildJacobianConstraint(core::MultiMatrixDerivId , const sofa::type::vector<sofa::type::Vec3> & , double , Index ) const override {}

    virtual void storeLambda(const core::ConstraintParams* , core::MultiVecDerivId , Index , Index , const sofa::linearalgebra::BaseVector* ) const override {}

};

}

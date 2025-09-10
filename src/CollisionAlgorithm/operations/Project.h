#pragma once

#include <CollisionAlgorithm/BaseProximity.h>
#include <CollisionAlgorithm/BaseOperation.h>
#include <CollisionAlgorithm/elements/PointElement.h>
#include <limits.h>

namespace sofa::collisionAlgorithm::Operations::Project {

struct Result {
    Result() {}
    Result(double d,const BaseProximity::SPtr & p) : distance(d), prox(p) {}

    double distance;
    BaseProximity::SPtr prox;
};

class Operation : public GenericOperation<Operation,//Type of the operation
                                          Result,//Default return type
                                          const type::Vec3 & , const BaseElement::SPtr & //Parameters
                                          > {
public:

    Result defaultFunc(const type::Vec3 & , const BaseElement::SPtr & ) const override {
        return Result(std::numeric_limits<double>::max(),NULL);
    }

    void notFound(const std::type_info & id) const override {
        std::cerr << "ERROR the operation ProjectOperation is not registered with for type = " << sofa::helper::NameDecoder::decodeFullName(id) << std::endl;
    }

};

typedef Operation::FUNC FUNC;

}

